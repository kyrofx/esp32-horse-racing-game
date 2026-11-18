#pragma once

#include <stdint.h>

namespace race {
namespace config {
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kLinkBaud = 115200;
constexpr uint16_t kHeartbeatMs = 80;
constexpr uint16_t kLinkTimeoutMs = 250;
constexpr uint16_t kFrameGapMs = 25;
constexpr uint16_t kAckRetryMs = 90;
constexpr uint8_t kCommandAttempts = 3;
constexpr uint16_t kStatusPeriodMs = 500;
constexpr uint16_t kStatusFreshMs = 1500;
constexpr uint16_t kConsoleReportMs = 1000;

constexpr uint16_t kButtonDebounceMs = 25;
constexpr uint16_t kLedUpdateMs = 80;
constexpr uint8_t kLedMask = 0x0f;
constexpr uint8_t kLedSweepActive = 0xff;
constexpr uint16_t kLedStepMs = 800;
constexpr uint32_t kI2cHz = 100000;
constexpr uint16_t kI2cTimeoutMs = 8;

constexpr uint8_t kTriggerUs = 12;
constexpr uint32_t kEchoTimeoutUs = 30000;
constexpr uint16_t kSonarMinMm = 20;
constexpr uint16_t kSonarMaxMm = 4000;
constexpr uint16_t kSonarSlotMs = 80;  // P1 then P2, one trigger per slot
constexpr uint16_t kSonarQuietMs = 65;
constexpr uint16_t kRemoteShotTimeoutMs = 150;
constexpr uint16_t kSampleFreshMs = 240;

constexpr uint16_t kPwmPeriodUs = 4000;  // 250 Hz, 1 us hardware ticks
constexpr uint8_t kMotorDutyPercent = 20;
constexpr uint16_t kMotorBurstMs = 120;
constexpr uint8_t kMaxDutyPercent = 25;
constexpr uint16_t kMaxMotorOnMs = 160;
constexpr uint16_t kMinMotorOffMs = 1500;  // shared by both bridges
constexpr uint8_t kButtonDuty[] = {25, 50, 75, 100};
// 0 inhibits, 1 releases, 2..5 select a button, 6 blocks overlapping presses.
inline uint8_t buttonDriveState(uint8_t mask) {
  switch (mask) {
    case 0: return 1;
    case 1: return 2;
    case 2: return 3;
    case 4: return 4;
    case 8: return 5;
    default: return 6;
  }
}
constexpr uint16_t kButtonDriveMs = 1000;
constexpr uint16_t kButtonPeriodUs = 25000;
constexpr uint16_t kConfirmMs = 20000;
constexpr uint32_t kObserveMs = 60000;
constexpr uint16_t kMotorSettleMs = 65;
constexpr uint16_t kMovementDeadlineMs = 630;
constexpr uint16_t kStationaryJitterMm = 5;
constexpr uint16_t kMovementMm = 12;
static_assert(kStationaryJitterMm < kMovementMm, "Movement must exceed allowed baseline jitter");
static_assert(kSonarQuietMs >= kEchoTimeoutUs / 1000, "Allow echoes to finish before the next ping");
static_assert(kSonarSlotMs >= kSonarQuietMs, "Leave a quiet interval between ultrasonic shots");
static_assert(2 * kSonarSlotMs < kSampleFreshMs, "Allow one full round of sonar slots");
static_assert(kMotorDutyPercent > 0 && kMotorDutyPercent <= kMaxDutyPercent, "Invalid test duty");
static_assert(kMotorBurstMs > 0 && kMotorBurstMs <= kMaxMotorOnMs, "Invalid test burst length");
static_assert(kMotorBurstMs * 1000UL % kPwmPeriodUs == 0, "Use a whole number of PWM periods");
}  // namespace config

// Unsigned subtraction handles the millis()/micros() wrap.
inline uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }

enum class Verdict : uint8_t { Manual, Pass, Fail };
inline const char* label(Verdict result) {
  switch (result) {
    case Verdict::Pass: return "PASS";
    case Verdict::Fail: return "FAIL";
    default: return "MANUAL CHECK";
  }
}

enum class RangeState : uint8_t { None, Valid, OutOfRange, Timeout, HardwareError };
struct RangeSample {
  constexpr RangeSample(RangeState state = RangeState::None, uint16_t mm = 0, uint16_t echoUs = 0)
      : state(state), mm(mm), echoUs(echoUs) {}
  RangeState state;
  uint16_t mm;
  uint16_t echoUs;
};

inline RangeSample measureEcho(uint16_t widthUs) {
  const uint16_t mm = (uint32_t(widthUs) * 343 + 1000) / 2000;  // about 343 m/s
  const RangeState state = mm >= config::kSonarMinMm && mm <= config::kSonarMaxMm
                               ? RangeState::Valid : RangeState::OutOfRange;
  return {state, mm, widthUs};
}
}  // namespace race
