#include "Link.h"

#include <string.h>

namespace race {
namespace wire {
namespace {
uint32_t get32(const uint8_t* in) {
  return uint32_t(get16(in)) | (uint32_t(get16(in + 2)) << 16);
}

void put32(uint8_t* out, uint32_t value) {
  put16(out, uint16_t(value));
  put16(out + 2, uint16_t(value >> 16));
}

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xffff;  // CRC-16/CCITT-FALSE
  for (size_t i = 0; i < length; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
  }
  return crc;
}

uint8_t payloadSize(Kind kind) {
  switch (kind) {
    case Kind::Ping: case Kind::LedState: case Kind::P1Button: return 1;
    case Kind::Ack: return 7;
    case Kind::Test: case Kind::Stop: case Kind::SampleRequest: case Kind::ButtonArm: return 0;
    case Kind::Sample: return 7;
    case Kind::Status: return kStatusSize;
    default: return 255;
  }
}

bool newer(uint16_t candidate, uint16_t ref) {
  const uint16_t diff = candidate - ref;
  return diff != 0 && diff < 0x8000;
}

// The largest raw packet is <254 bytes, so COBS never needs a 0xff run.
bool decode(const uint8_t* in, size_t size, Frame& out) {
  uint8_t raw[kMaxRaw];
  size_t r = 0, w = 0;
  while (r < size) {
    const uint8_t code = in[r++];
    if (code == 0 || code == 255 || r + code - 1 > size || w + code - 1 > sizeof(raw)) return false;
    for (uint8_t j = 1; j < code; ++j) raw[w++] = in[r++];
    if (r < size) {
      if (w >= sizeof(raw)) return false;
      raw[w++] = 0;
    }
  }
  if (w < kHeader + 2 || raw[0] != kVersion || raw[2] > uint8_t(Role::Nano) || raw[9] > kMaxPayload ||
      w != kHeader + raw[9] + 2 || get16(raw + w - 2) != crc16(raw, w - 2)) return false;
  const Kind kind = static_cast<Kind>(raw[1]);
  const uint32_t session = get32(raw + 3);
  if (!session || payloadSize(kind) != raw[9]) return false;
  out = {kind, static_cast<Role>(raw[2]), session, get16(raw + 7), raw[9], {}};
  memcpy(out.data, raw + kHeader, out.len);
  return true;
}
}  // namespace

void put16(uint8_t* out, uint16_t value) {
  out[0] = uint8_t(value);
  out[1] = uint8_t(value >> 8);
}

uint16_t get16(const uint8_t* in) { return uint16_t(in[0]) | (uint16_t(in[1]) << 8); }

size_t encode(const Frame& frame, uint8_t (&out)[kMaxFrame]) {
  if (frame.len > kMaxPayload || frame.len != payloadSize(frame.kind) || !frame.session ||
      uint8_t(frame.source) > uint8_t(Role::Nano)) return 0;
  uint8_t raw[kMaxRaw] = {kVersion, uint8_t(frame.kind), uint8_t(frame.source)};
  put32(raw + 3, frame.session);
  put16(raw + 7, frame.seq);
  raw[9] = frame.len;
  memcpy(raw + kHeader, frame.data, frame.len);
  const size_t end = kHeader + frame.len;
  put16(raw + end, crc16(raw, end));

  out[0] = 0;
  size_t codeAt = 1, w = 2;
  uint8_t code = 1;
  for (size_t r = 0; r < end + 2; ++r) {
    if (raw[r] == 0) {
      out[codeAt] = code;
      codeAt = w++;
      code = 1;
    } else {
      out[w++] = raw[r];
      ++code;
    }
  }
  out[codeAt] = code;
  out[w++] = 0;
  return w;
}

bool Decoder::push(uint8_t byte, uint32_t now, Frame& out) {
  if (size_ && elapsed(now, lastByte_) >= config::kFrameGapMs) {
    ++errors_;
    size_ = 0;
    discard_ = true;
  }
  lastByte_ = now;
  if (byte == 0) {
    const bool complete = inFrame_ && !discard_ && size_;
    const bool good = complete && decode(data_, size_, out);
    if (complete && !good) ++errors_;
    inFrame_ = true;
    discard_ = false;
    size_ = 0;
    return good;
  }
  if (!inFrame_ || discard_) return false;
  if (size_ == sizeof(data_)) {
    ++errors_;
    discard_ = true;
    size_ = 0;
  } else {
    data_[size_++] = byte;
  }
  return false;
}

bool Link::peerMaySend(Kind kind) const {
  if (kind == Kind::Ping || kind == Kind::Ack || kind == Kind::Sample) return true;
  if (role_ == Role::Nano) {
    return kind == Kind::Test || kind == Kind::Stop || kind == Kind::SampleRequest || kind == Kind::LedState || kind == Kind::P1Button || kind == Kind::ButtonArm;
  }
  return kind == Kind::Status;
}

bool Link::receive(uint8_t byte, uint32_t now, Frame& out) {
  if (!decoder_.push(byte, now, out)) return false;
  // A short between TX and RX can otherwise answer our own pings and produce a false PASS.
  if (out.session == session_) { ++selfEchoes_; return false; }
  if (out.source == role_) { ++wrongRoleFrames_; return false; }
  if (!peerMaySend(out.kind)) return false;
  if (out.kind == Kind::Ping && (out.data[0] > 1 || (out.source == Role::Nano && out.data[0]))) return false;
  if (out.kind == Kind::P1Button && out.data[0] > 6) return false;
  if (out.kind == Kind::LedState && out.data[0] > config::kLedMask) return false;
  if (peerSession_ != out.session) {
    if (peerSession_ && pending_.active) finish(false);
    peerSession_ = out.session;
    heardPing_ = heardAck_ = peerPermit_ = pingAcked_ = havePeerSeq_ = haveApplied_ = false;
  }

  if (out.kind == Kind::Test || out.kind == Kind::Stop) {
    if (!haveApplied_ || newer(out.seq, appliedSeq_)) return true;
    if (appliedSeq_ == out.seq && appliedKind_ == out.kind) acknowledge(out);
    return false;
  }
  if (out.kind == Kind::Ping && heardPing_ && elapsed(now, pingRxAt_) >= config::kLinkTimeoutMs) {
    // A long disconnection can span half the sequence space. Re-sync but require a new ACK.
    havePeerSeq_ = heardAck_ = false;
  }
  if (havePeerSeq_ && !newer(out.seq, peerSeq_)) return false;
  havePeerSeq_ = true;
  peerSeq_ = out.seq;

  if (out.kind == Kind::Ping) {
    heardPing_ = true;
    peerPermit_ = out.data[0] != 0;
    pingRxAt_ = now;
    acknowledge(out);
    return false;
  }
  if (out.kind == Kind::Ack) {
    if (get32(out.data + 3) != session_) return false;
    const uint16_t seq = get16(out.data + 1);
    const Kind kind = static_cast<Kind>(out.data[0]);
    if (kind == Kind::Ping && sentPing_ && !pingAcked_ && seq == pingSeq_ &&
        elapsed(now, pingTxAt_) < config::kLinkTimeoutMs) {
      heardAck_ = pingAcked_ = true;
      ackRxAt_ = now;
    } else if (pending_.active && pending_.attempts && pending_.kind == kind && pending_.seq == seq) {
      finish(true);
    }
    return false;
  }
  return true;
}

bool Link::emit(Kind kind, uint16_t seq, const uint8_t* data, uint8_t len) {
  if (len > kMaxPayload) return false;
  Frame frame{kind, role_, session_, seq, len, {}};
  if (len) memcpy(frame.data, data, len);
  uint8_t packet[kMaxFrame];
  const size_t count = encode(frame, packet);
  return count && writer_(context_, packet, count);
}

bool Link::send(Kind kind, const uint8_t* data, uint8_t len, uint16_t* sequence) {
  if (!emit(kind, nextSeq_, data, len)) return false;
  if (sequence) *sequence = nextSeq_;
  ++nextSeq_;
  return true;
}

void Link::acknowledge(const Frame& frame) {
  if (frame.kind == Kind::Test || frame.kind == Kind::Stop) {
    haveApplied_ = true;
    appliedSeq_ = frame.seq;
    appliedKind_ = frame.kind;
  }
  uint8_t data[7] = {uint8_t(frame.kind)};
  put16(data + 1, frame.seq);
  put32(data + 3, frame.session);
  send(Kind::Ack, data, sizeof(data));
}

bool Link::request(Kind kind, uint32_t now) {
  if (role_ != Role::Node || (kind != Kind::Test && kind != Kind::Stop)) return false;
  if (pending_.active && kind != Kind::Stop) return false;
  pending_ = {kind, nextCommandSeq_++, 0, now, now, true};
  return true;
}

void Link::finish(bool acked) {
  receipt_ = {pending_.kind, acked};
  pending_.active = false;
  hasReceipt_ = true;
}

bool Link::takeReceipt(Receipt& out) {
  if (!hasReceipt_) return false;
  out = receipt_;
  hasReceipt_ = false;
  return true;
}

void Link::tick(uint32_t now, bool permit) {
  if (pending_.active) {
    if (elapsed(now, pending_.start) >= config::kAckRetryMs * config::kCommandAttempts) {
      finish(false);
    } else if (pending_.attempts == 0 ||
               (pending_.attempts < config::kCommandAttempts &&
                elapsed(now, pending_.lastTx) >= config::kAckRetryMs)) {
      if (emit(pending_.kind, pending_.seq, nullptr, 0)) {
        ++pending_.attempts;
        pending_.lastTx = now;
      }
    }
  }
  if (!sentPing_ || elapsed(now, pingTxAt_) >= config::kHeartbeatMs) {
    const uint8_t data = permit ? 1 : 0;
    const uint16_t seq = nextSeq_;
    if (send(Kind::Ping, &data, 1)) {
      sentPing_ = true;
      pingAcked_ = false;
      pingSeq_ = seq;
      pingTxAt_ = now;
    }
  }
}

bool Link::online(uint32_t now) const {
  return heardPing_ && heardAck_ && elapsed(now, pingRxAt_) < config::kLinkTimeoutMs &&
         elapsed(now, ackRxAt_) < config::kLinkTimeoutMs;
}

void packSample(const RangeSample& sample, uint8_t (&out)[7], uint16_t request) {
  out[0] = uint8_t(sample.state);
  put16(out + 1, sample.mm);
  put16(out + 3, sample.echoUs);
  put16(out + 5, request);
}

bool unpackSample(const Frame& frame, RangeSample& out) {
  if (frame.kind != Kind::Sample || frame.len != 7) return false;
  const auto state = static_cast<RangeState>(frame.data[0]);
  const uint16_t mm = get16(frame.data + 1), us = get16(frame.data + 3);
  if (state == RangeState::Valid || state == RangeState::OutOfRange) {
    const RangeSample expected = measureEcho(us);
    if (us > config::kEchoTimeoutUs || mm != expected.mm || state != expected.state) return false;
  } else if ((state != RangeState::Timeout && state != RangeState::HardwareError) || mm != 0 || us != 0) {
    return false;
  }
  out = {state, mm, us};
  return true;
}
}  // namespace wire
}  // namespace race
