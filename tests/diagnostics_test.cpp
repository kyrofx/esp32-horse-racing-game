#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>

#include <race/ButtonPairing.h>
#include <race/ButtonTracker.h>
#include <race/Link.h>
#include <race/MotorTrial.h>

namespace {
using namespace race;
using namespace race::wire;
using State = MotorTrial::State;

void framing() {
  // Independent CRC-16/CCITT-FALSE check vector for a STOP frame, session 0x12345678, seq 0.
  const Frame original{Kind::Stop, Role::Node, 0x12345678, 0, 0, {}};
  const uint8_t expected[] = {0, 3, 4, 4, 5, 0x78, 0x56, 0x34, 0x12, 1, 1, 3, 0x7d, 0x49, 0};
  uint8_t encoded[kMaxFrame];
  const size_t n = encode(original, encoded);
  assert(n == sizeof(expected));
  assert(memcmp(encoded, expected, n) == 0);
  Decoder parser;
  Frame got{};
  auto push = [&](const uint8_t* data, size_t len, uint32_t now) {
    size_t hits = 0;
    for (size_t i = 0; i < len; ++i) if (parser.push(data[i], now, got)) ++hits;
    return hits;
  };
  const uint8_t noise[] = {0xff, 0x01, 0x4e};
  assert(push(noise, sizeof(noise), 0) == 0);
  assert(push(encoded, n, 1) == 1);
  assert(got.kind == original.kind && got.source == Role::Node && got.seq == 0 && got.session == original.session);

  encoded[5] ^= 2;
  assert(push(encoded, n, 2) == 0 && parser.errors() == 1);
  encoded[5] ^= 2;
  assert(push(encoded, n - 3, 3) == 0);
  assert(push(encoded + n - 3, 3, 3 + config::kFrameGapMs) == 0 && parser.errors() == 2);
  for (size_t i = 0; i < 2 * kMaxFrame; ++i) assert(!parser.push(1, 40, got));
  assert(parser.errors() == 3 && push(encoded, n, 41) == 1);
}

struct Line {
  bool cut = false;
  Kind dropNext{};
  std::deque<uint8_t> queue;

  static bool send(void* context, const uint8_t* data, size_t n) {
    auto& line = *static_cast<Line*>(context);
    if (line.cut) return true;  // UART can transmit even with its wire removed
    Decoder parser;
    Frame frame{};
    assert(n >= 2);
    for (size_t i = 0; i < n - 1; ++i) assert(!parser.push(data[i], 0, frame));
    assert(parser.push(data[n - 1], 0, frame));
    if (frame.kind == line.dropNext) { line.dropNext = {}; return true; }
    line.queue.insert(line.queue.end(), data, data + n);
    return true;
  }
};

struct Pair {
  Line toNode, toNano;
  Link node{Role::Node, Line::send, &toNano};
  Link nano{Role::Nano, Line::send, &toNode};
  unsigned tests = 0, stops = 0;
  unsigned ledUpdates = 0;
  uint8_t ledMask = 0, reportedLedMask = config::kLedSweepActive;

  Pair() { node.begin(0x12345678); nano.begin(0xaabbccdd); }

  void receive(Link& link, Line& line, uint32_t now) {
    while (!line.queue.empty()) {
      const uint8_t byte = line.queue.front();
      line.queue.pop_front();
      Frame frame{};
      if (!link.receive(byte, now, frame)) continue;
      if (frame.kind == Kind::Test) { ++tests; link.acknowledge(frame); }
      if (frame.kind == Kind::Stop) { ++stops; link.acknowledge(frame); }
      if (frame.kind == Kind::LedState) {
        ++ledUpdates;
        ledMask = frame.data[0];
      }
      if (frame.kind == Kind::Status) reportedLedMask = frame.data[6];
    }
  }

  void run(uint32_t from, uint32_t until, bool permit = true) {
    for (uint32_t now = from; now < until; now += 5) {
      node.tick(now, permit);
      nano.tick(now);
      receive(nano, toNano, now);
      receive(node, toNode, now);
    }
  }
};

void ledStatePackets() {
  Pair pair;
  pair.run(0, 200);
  uint32_t now = 200;
  const uint8_t masks[] = {1, 2, 4, 8, 0x0a};
  for (const uint8_t mask : masks) {
    assert(pair.node.send(Kind::LedState, &mask, 1));
    pair.run(now, now + 5);
    assert(pair.ledMask == mask);
    now += 5;
  }
  assert(pair.ledUpdates == sizeof(masks));

  const uint8_t reported[kStatusSize] = {0, 0, 0, 0, 0, 0, pair.ledMask};
  assert(pair.nano.send(Kind::Status, reported, sizeof(reported)));
  pair.run(now, now + 5);
  assert(pair.reportedLedMask == 0x0a);
  now += 5;

  const uint8_t released = 0;
  pair.toNano.dropNext = Kind::LedState;
  assert(pair.node.send(Kind::LedState, &released, 1));
  pair.run(now, now + config::kLedUpdateMs);
  now += config::kLedUpdateMs;
  assert(pair.ledMask == 0x0a);
  assert(pair.node.send(Kind::LedState, &released, 1));
  pair.run(now, now + 5);
  now += 5;
  assert(pair.ledUpdates == sizeof(masks) + 1 && pair.ledMask == 0);

  const uint8_t invalid = 0x10;
  assert(pair.node.send(Kind::LedState, &invalid, 1));
  assert(pair.nano.send(Kind::LedState, &masks[0], 1));
  pair.run(now, now + 5);
  assert(pair.ledUpdates == sizeof(masks) + 1 && pair.ledMask == 0);
}

void physicalButtonPairing() {
  using Event = ButtonPairing::Event;
  ButtonPairing pairing;
  const ButtonMapping blank;
  assert(!blank.valid() && blank.ledMask(0, 0x0f) == 0 && blank.ledMask(1, 0x0f) == 0);
  pairing.start();
  assert(pairing.update(1, true) == Event::None);  // held before the first prompt
  assert(pairing.update(0, true) == Event::None && pairing.step() == 0);
  assert(pairing.update(0x10, true) == Event::WrongPlayer);
  assert(pairing.update(0, true) == Event::None);
  assert(pairing.update(3, true) == Event::Multiple);
  assert(pairing.update(1, true) == Event::None);  // both must release before retrying
  assert(pairing.update(0, true) == Event::None && pairing.step() == 0);

  assert(pairing.update(4, true) == Event::None);
  assert(pairing.update(5, true) == Event::Multiple);  // a second press before release
  assert(pairing.update(0, true) == Event::None && pairing.step() == 0);

  const uint8_t sequence[] = {2, 0, 3, 1, 3, 1, 2, 0};
  for (uint8_t step = 0; step < sizeof(sequence); ++step) {
    assert(pairing.step() == step);
    const uint8_t button = 1u << ((step / 4) * 4 + sequence[step]);
    if (step == 1) {
      pairing.update(0, true);
      assert(pairing.update(4, true) == Event::Reused);
    }
    if (step == 4) {
      assert(pairing.update(0, true) == Event::None);
      assert(pairing.update(button, true) == Event::None);
      assert(pairing.update(button, false) == Event::None);  // Nano stopped reporting the chosen LED
      assert(pairing.update(button, true) == Event::None);
      assert(pairing.update(0, true) == Event::None && pairing.step() == 4);
    }
    assert(pairing.update(0, true) == Event::None);
    assert(pairing.update(button, true) == Event::None && pairing.step() == step);
    assert(pairing.update(button, true) == Event::None);
    assert(pairing.update(0, true) == Event::Assigned);
    assert(pairing.draft().inputForLed[step] == sequence[step]);
  }
  assert(!pairing.active() && pairing.draft().valid());
  const auto mapping = pairing.draft();
  assert(mapping.ledMask(0, 0x05) == 0x03);  // P1 inputs 1 and 3 select LEDs 2 and 1
  assert(mapping.ledMask(1, 0x09) == 0x09);  // P2 inputs 1 and 4 select LEDs 4 and 1
  assert(mapping.ledMask(1, 0) == 0);

  auto corrupt = mapping;
  corrupt.inputForLed[1] = corrupt.inputForLed[0];
  assert(!corrupt.valid());
  pairing.start();
  assert(pairing.update(0, true) == Event::None);
  assert(pairing.update(1, true) == Event::None);
  pairing.cancel();
  assert(!pairing.active() && !pairing.draft().valid() && mapping.valid());
}

void linkHeartbeat() {
  Pair pair;
  assert(!pair.node.online(0) && !pair.nano.permitsMotion(0));
  pair.toNode.cut = true;
  pair.run(0, 300);
  assert(pair.nano.heardPeer() && !pair.nano.gotAck());
  assert(!pair.node.online(299) && !pair.nano.permitsMotion(299));
  pair.toNode.cut = false;
  pair.run(300, 500, false);
  assert(pair.node.online(499) && pair.nano.online(499) && !pair.nano.permitsMotion(499));
  pair.run(500, 600);
  assert(pair.nano.permitsMotion(599));
  pair.toNano.cut = true;
  pair.run(600, 900);
  assert(!pair.nano.permitsMotion(899) && !pair.node.online(899));
  pair.toNano.cut = false;
  pair.run(900, 1100);
  assert(pair.nano.permitsMotion(1099) && pair.node.online(1099));

  Frame ping{Kind::Ping, Role::Node, 0xfeedbeef, 65535, 1, {1}};
  auto feed = [&](uint32_t now) {
    uint8_t bytes[kMaxFrame];
    const size_t n = encode(ping, bytes);
    Frame got{};
    for (size_t i = 0; i < n; ++i) pair.nano.receive(bytes[i], now, got);
  };
  feed(1100);  // changed boot session must invalidate the old heartbeat round trip
  assert(pair.nano.peerSession() == ping.session && !pair.nano.online(1100) && pair.nano.heardPeer());
  pair.toNode.queue.clear();
  ping.seq = 0;
  feed(1101);
  assert(!pair.toNode.queue.empty());  // acknowledged sequence wrap
  pair.toNode.queue.clear();
  ping.seq = 65535;
  feed(1102);
  assert(pair.toNode.queue.empty());  // discarded old sequence
}

void loopbackRejection() {
  Line echo;
  Link node{Role::Node, Line::send, &echo};
  node.begin(0x11223344);
  for (uint32_t now = 0; now < 1000; now += config::kHeartbeatMs) {
    node.tick(now);
    while (!echo.queue.empty()) {
      const uint8_t byte = echo.queue.front();
      echo.queue.pop_front();
      Frame frame{};
      assert(!node.receive(byte, now, frame));
    }
    assert(!node.online(now) && node.peerSession() == 0);
  }
  assert(node.selfEchoes() > 0 && node.wrongRoleFrames() == 0);
  const Frame otherNode{Kind::Ping, Role::Node, 0x55667788, 0, 1, {1}};
  uint8_t bytes[kMaxFrame];
  const size_t n = encode(otherNode, bytes);
  Frame out{};
  for (size_t i = 0; i < n; ++i) assert(!node.receive(bytes[i], 1000, out));
  assert(node.wrongRoleFrames() == 1 && !node.heardPeer() && node.peerSession() == 0);
}

void reliableCommands() {
  Pair pair;
  pair.run(0, 200);
  pair.toNano.dropNext = Kind::Test;
  assert(pair.node.request(Kind::Test, 200));
  pair.run(200, 285);
  assert(pair.tests == 0);
  // Drop the next transmission's ACK but leave the heartbeat channel usable.
  pair.toNode.dropNext = Kind::Ack;
  pair.run(290, 375);
  assert(pair.tests == 1);
  Link::Receipt got{};
  assert(!pair.node.takeReceipt(got));
  pair.run(375, 475);
  assert(pair.node.takeReceipt(got) && got.kind == Kind::Test && got.acked && pair.tests == 1);

  assert(pair.node.request(Kind::Test, 475));
  assert(pair.node.request(Kind::Stop, 475));
  pair.run(475, 550, false);
  assert(pair.stops == 1 && pair.tests == 1 && !pair.nano.permitsMotion(549));
  assert(pair.node.takeReceipt(got) && got.kind == Kind::Stop && got.acked);

  pair.toNano.cut = true;
  assert(pair.node.request(Kind::Test, 550));
  pair.run(550, 850, false);
  assert(pair.node.takeReceipt(got) && got.kind == Kind::Test && !got.acked);
  pair.toNano.cut = false;
  pair.run(850, 1100, false);
  assert(pair.tests == 1);  // no application after the retry window expires
}

void buttonEvidence() {
  ButtonTracker button;
  const auto d = config::kButtonDebounceMs;
  button.reset(true, 0);
  assert(!button.passed());
  assert(!button.update(false, 1));
  assert(button.update(false, 1 + d) && !button.passed());  // released boot-held button
  assert(!button.update(true, 100));
  assert(!button.update(false, 102));  // bounce
  assert(!button.update(false, 102 + d));
  assert(!button.update(true, UINT32_MAX - 8));
  assert(button.update(true, UINT32_MAX - 8 + uint32_t(d)) && button.pressed() && !button.passed());
  assert(!button.update(false, 80));
  assert(button.update(false, 80 + d) && !button.pressed() && button.passed());
}

void p1DutyMapping() {
  assert(config::buttonDriveState(0) == 1);
  for (uint8_t i = 0; i < 4; ++i) {
    const auto state = config::buttonDriveState(1u << i);
    assert(state == i + 2 && config::kButtonDuty[state - 2] == 25 * (i + 1));
  }
  for (uint8_t mask = 1; mask < 16; ++mask)
    if (mask & (mask - 1)) assert(config::buttonDriveState(mask) == 6);
}

void sonarResults() {
  assert(measureEcho(90).state == RangeState::OutOfRange);
  assert(measureEcho(117).mm == 20 && measureEcho(117).state == RangeState::Valid);
  assert(measureEcho(5831).mm == 1000);
  assert(measureEcho(25000).state == RangeState::OutOfRange);
  Frame frame{Kind::Sample, Role::Nano, 1, 1, 7, {}};
  uint8_t samplePayload[7]{};
  packSample(measureEcho(5831), samplePayload, 65535);
  memcpy(frame.data, samplePayload, sizeof(samplePayload));
  RangeSample got;
  assert(unpackSample(frame, got) && got.state == RangeState::Valid && got.mm == 1000);
  assert(get16(frame.data + 5) == 65535);
  frame.data[1] ^= 1;
  assert(!unpackSample(frame, got));
  packSample({RangeState::Timeout}, samplePayload);
  memcpy(frame.data, samplePayload, sizeof(samplePayload));
  assert(unpackSample(frame, got) && got.state == RangeState::Timeout);
  frame.data[3] = 2;
  assert(!unpackSample(frame, got));
}

struct Bridge {
  unsigned starts = 0;
  uint8_t motor = 255;
  bool active = false, reverse = false, failStart = false;
  static bool burst(void* context, uint8_t motor, bool rev) {
    auto& self = *static_cast<Bridge*>(context);
    assert(!self.active);
    self.active = true;
    self.motor = motor;
    self.reverse = rev;
    ++self.starts;
    return !self.failStart;
  }
  static void off(void* context) { static_cast<Bridge*>(context)->active = false; }
};

constexpr BenchSetup bench{true, 5000, 150, {{"fixture 1", 6000, 100, 600}, {"fixture 2", 6000, 100, 600}}};
constexpr uint32_t kNow = 2000;
constexpr uint32_t kToken = 0x11223344;

struct Rig {
  Bridge bridge;
  MotorTrial trial;
  Rig(const BenchSetup& setup = bench, bool ready = true) : trial(setup, {Bridge::burst, Bridge::off, &bridge}) {
    trial.begin(0, ready);
  }
  void range(uint32_t now, uint8_t motor, uint16_t mm) { trial.sample(now, motor, {RangeState::Valid, mm}); }
  void baseline(uint32_t now, uint8_t motor = 0, uint16_t mm = 300) {
    range(now - 180, motor, mm);
    range(now - 20, motor, mm + 1);
  }
  void start(uint32_t now = kNow, uint8_t motor = 0, bool reverse = false) {
    baseline(now, motor);
    assert(!trial.prepare(now, motor, reverse, kToken, true));
    assert(!trial.confirm(now, kToken, true));
    assert(bridge.active && bridge.motor == motor && bridge.reverse == reverse);
  }
};

void motorPreconditions() {
  const BenchSetup blank{};
  Rig locked(blank);
  locked.baseline(kNow);
  assert(locked.trial.prepare(kNow, 0, false, kToken, true) && !locked.bridge.active);
  auto overvoltage = bench;
  overvoltage.measuredSupplyMv = 7000;
  Rig unsafe(overvoltage);
  assert(unsafe.trial.lockReason());
  Rig badDriver(bench, false);
  assert(badDriver.trial.lockReason());

  Rig rig;
  assert(rig.trial.prepare(1000, 0, false, kToken, true));
  assert(rig.trial.prepare(kNow, 0, false, kToken, true));  // no baseline
  rig.baseline(kNow, 0, 101);
  assert(rig.trial.prepare(kNow, 0, false, kToken, true));  // at boundary
  rig.range(kNow - 20, 0, 310);
  rig.range(kNow, 0, 310);
  assert(rig.trial.prepare(kNow, 0, false, kToken, true));  // too close in time
  rig.baseline(kNow);
  assert(rig.trial.prepare(kNow, 0, false, kToken, false));
  assert(!rig.trial.prepare(kNow, 0, false, kToken, true) && !rig.bridge.active);
  assert(rig.trial.confirm(kNow, kToken ^ 1, true));
  assert(rig.trial.confirm(kNow, kToken, true));  // wrong code consumed the request
  assert(!rig.trial.prepare(kNow, 0, false, kToken, true));
  rig.baseline(kNow + config::kConfirmMs);
  assert(rig.trial.confirm(kNow + config::kConfirmMs, kToken, true));  // expired despite fresh range
  assert(rig.bridge.starts == 0);
}

void motorEvidenceAndCooldown() {
  Rig rig;
  uint32_t now = kNow;
  for (uint8_t motor = 0; motor < 2; ++motor) {
    for (bool reverse : {false, true}) {
      rig.start(now, motor, reverse);
      assert(rig.trial.confirm(now, kToken, true));  // never fire twice
      assert(!rig.trial.recordObservation(now, true));
      rig.trial.tick(now + config::kMotorBurstMs - 1, true);
      assert(rig.bridge.active);
      const uint32_t off = now + config::kMotorBurstMs;
      rig.trial.tick(off, true);
      assert(rig.trial.state() == State::Verify && !rig.bridge.active);
      const uint16_t moved = reverse ? 270 : 330;
      rig.range(off + 1, motor, moved);  // still settling; not evidence
      rig.range(off + 100, motor, moved);
      rig.range(off + 101, motor, moved);  // two buffered reports too close together
      assert(rig.trial.state() == State::Verify);
      rig.range(off + 260, motor, moved);
      assert(rig.trial.state() == State::Observe && rig.trial.direction(motor, reverse) == Verdict::Manual);
      assert(rig.trial.recordObservation(off + 261, true));
      assert(rig.trial.direction(motor, reverse) == Verdict::Pass);
      assert(rig.trial.verdict(motor) == (reverse ? Verdict::Pass : Verdict::Manual));
      rig.baseline(off + 400, motor ^ 1);
      assert(rig.trial.prepare(off + 400, motor ^ 1, false, kToken, true));  // shared cooldown
      now = off + config::kMinMotorOffMs;
    }
  }
  assert(rig.bridge.starts == 4 && !rig.bridge.active);
}

void motorFailures() {
  for (int condition = 0; condition < 7; ++condition) {
    Rig rig;
    if (condition == 6) {
      rig.bridge.failStart = true;
      rig.baseline(kNow);
      assert(!rig.trial.prepare(kNow, 0, false, kToken, true));
      assert(rig.trial.confirm(kNow, kToken, true));
    } else {
      rig.start();
      if (condition == 0) rig.trial.sample(kNow + 10, 0, {RangeState::Timeout});
      if (condition == 1) rig.range(kNow + 10, 0, 601);
      if (condition == 2) rig.trial.tick(kNow + 10, false);  // lost link/console/permission
      if (condition == 3) rig.trial.stop(kNow + 10, "STOP");
      if (condition == 4) rig.trial.tick(kNow + config::kMaxMotorOnMs + 1, true);
      if (condition == 5) {
        const uint32_t off = kNow + config::kMotorBurstMs;
        rig.trial.tick(off, true);
        for (uint32_t dt = 120; dt < config::kMovementDeadlineMs; dt += 160) {
          rig.range(off + dt, 0, 300);
          rig.trial.tick(off + dt, true);
        }
        rig.trial.tick(off + config::kMovementDeadlineMs, true);
      }
    }
    assert(rig.trial.state() == State::Fault && rig.trial.verdict(0) == Verdict::Fail && !rig.bridge.active);
    for (uint32_t now = kNow + 3000; now < kNow + 4000; now += 100) {
      rig.range(now, 1, 300);
      rig.trial.tick(now, true);
      assert(rig.trial.prepare(now, 1, true, kToken, true));
      assert(rig.trial.confirm(now, kToken, true));
    }
    assert(rig.bridge.starts == 1);  // never retries/reverses/moves other bridge on failure
    assert(rig.trial.clearFault() && rig.trial.verdict(0) == Verdict::Fail && !rig.bridge.active);
  }

  Rig stale;
  stale.baseline(kNow);
  const uint32_t nearExpiry = kNow + config::kSampleFreshMs - 25;
  assert(!stale.trial.prepare(nearExpiry, 0, false, kToken, true));
  assert(!stale.trial.confirm(nearExpiry, kToken, true));
  stale.trial.tick(nearExpiry + 5, true);
  assert(stale.trial.state() == State::Fault && !stale.bridge.active);

  Rig wrongDirection;
  wrongDirection.start();
  const uint32_t off = kNow + config::kMotorBurstMs;
  wrongDirection.trial.tick(off, true);
  wrongDirection.range(off + 100, 0, 330);
  wrongDirection.range(off + 260, 0, 330);
  assert(wrongDirection.trial.recordObservation(off + 261, false));
  assert(wrongDirection.trial.state() == State::Fault && wrongDirection.trial.verdict(0) == Verdict::Fail);
}
}  // namespace

int main() {
  const struct Test { const char* name; void (*run)(); } tests[] = {
      {"framing, CRC and resynchronization", framing},
      {"bidirectional heartbeat, sequence wrap and peer reboot", linkHeartbeat},
      {"P2 LED updates, applied-state report and role validation", ledStatePackets},
      {"interactive pairing with arbitrary order and release guards", physicalButtonPairing},
      {"self-echo and wrong-role heartbeat rejection", loopbackRejection},
      {"retries, duplicate suppression, STOP priority and ACK timeout", reliableCommands},
      {"button evidence, debounce and clock rollover", buttonEvidence},
      {"P1 duty selection and overlapping presses", p1DutyMapping},
      {"sonar conversion, result schema and timeout", sonarResults},
      {"motor lockout, fresh baseline and one-shot confirmation", motorPreconditions},
      {"four motor directions, manual evidence and shared cooldown", motorEvidenceAndCooldown},
      {"motor faults shut off outputs and never retry", motorFailures},
  };
  for (const auto& test : tests) {
    test.run();
    std::printf("PASS: %s\n", test.name);
  }
}
