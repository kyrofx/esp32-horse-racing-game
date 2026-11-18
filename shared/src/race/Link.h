#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Config.h"

namespace race {
namespace wire {
enum class Kind : uint8_t { Ping = 1, Ack, Test, Stop, SampleRequest, Sample, Status, LedState, P1Button, ButtonArm };
enum class Role : uint8_t { Node, Nano };
constexpr uint8_t kVersion = 4;
constexpr size_t kStatusSize = 8;
constexpr size_t kMaxPayload = kStatusSize;
constexpr size_t kHeader = 10;
constexpr size_t kMaxRaw = kHeader + kMaxPayload + 2;
constexpr size_t kMaxEncoded = kMaxRaw + 1;
constexpr size_t kMaxFrame = kMaxEncoded + 2;  // zero delimiters at both ends

struct Frame {
  Kind kind;
  Role source;
  uint32_t session;
  uint16_t seq;
  uint8_t len;
  uint8_t data[kMaxPayload];
};

void put16(uint8_t* out, uint16_t value);
uint16_t get16(const uint8_t* in);
size_t encode(const Frame& frame, uint8_t (&out)[kMaxFrame]);

class Decoder {
 public:
  bool push(uint8_t byte, uint32_t now, Frame& out);
  uint32_t errors() const { return errors_; }

 private:
  uint8_t data_[kMaxEncoded]{};
  size_t size_ = 0;
  uint32_t lastByte_ = 0, errors_ = 0;
  bool inFrame_ = false, discard_ = false;
};

// Used by firmware with a nonblocking UART writer and by the host tests with two queues.
class Link {
 public:
  using Send = bool (*)(void*, const uint8_t*, size_t);
  struct Receipt { Kind kind; bool acked; };

  Link(Role role, Send writer, void* context) : role_(role), writer_(writer), context_(context) {}
  void begin(uint32_t session) { session_ = session ? session : 1; }
  bool receive(uint8_t byte, uint32_t now, Frame& out);
  void tick(uint32_t now, bool permit = false);

  bool send(Kind kind, const uint8_t* data, uint8_t len, uint16_t* sequence = nullptr);
  bool request(Kind kind, uint32_t now);  // reliable Node -> Nano Test / Stop
  void acknowledge(const Frame& applied);
  bool takeReceipt(Receipt& out);

  bool online(uint32_t now) const;
  bool permitsMotion(uint32_t now) const { return online(now) && peerPermit_; }
  bool heardPeer() const { return heardPing_; }
  bool gotAck() const { return heardAck_; }
  uint32_t peerSession() const { return peerSession_; }
  uint32_t errors() const { return decoder_.errors(); }
  uint32_t selfEchoes() const { return selfEchoes_; }
  uint32_t wrongRoleFrames() const { return wrongRoleFrames_; }

 private:
  bool peerMaySend(Kind kind) const;
  bool emit(Kind kind, uint16_t seq, const uint8_t* data, uint8_t len);
  void finish(bool acked);

  Role role_;
  Send writer_;
  void* context_;
  Decoder decoder_;
  uint32_t session_ = 1, peerSession_ = 0;
  uint32_t selfEchoes_ = 0, wrongRoleFrames_ = 0;
  uint16_t nextSeq_ = 0, nextCommandSeq_ = 0;
  uint32_t pingTxAt_ = 0, pingRxAt_ = 0, ackRxAt_ = 0;
  uint16_t pingSeq_ = 0, peerSeq_ = 0;
  bool sentPing_ = false, heardPing_ = false, heardAck_ = false;
  bool pingAcked_ = false, peerPermit_ = false, havePeerSeq_ = false;
  bool haveApplied_ = false;
  uint16_t appliedSeq_ = 0;
  Kind appliedKind_{};
  struct Pending {
    Kind kind;
    uint16_t seq;
    uint8_t attempts;
    uint32_t start, lastTx;
    bool active;
  } pending_{};
  Receipt receipt_{};
  bool hasReceipt_ = false;
};

void packSample(const RangeSample& sample, uint8_t (&out)[7], uint16_t request = 0);
bool unpackSample(const Frame& frame, RangeSample& out);
}  // namespace wire
}  // namespace race
