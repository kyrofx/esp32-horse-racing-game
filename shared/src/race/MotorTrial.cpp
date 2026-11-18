#include "MotorTrial.h"

#include <stdlib.h>

namespace race {
const char* BenchSetup::lockReason() const {
  if (!enablePinsWired) return "ENA/ENB PWM wiring has not been configured";
  if (!measuredSupplyMv || !supplyLimitSettingMa) return "enter measured supply voltage and external current limit setting";
  for (const MotorSpec& spec : motors) {
    if (!spec.model || !spec.model[0] || !spec.ratedMv) return "enter both exact motor models and voltage ratings";
    if (measuredSupplyMv > spec.ratedMv) return "supply voltage exceeds a motor rating";
    if (spec.minMm < config::kSonarMinMm || spec.maxMm > config::kSonarMaxMm ||
        spec.maxMm <= spec.minMm + 2 * config::kMovementMm) return "configure both safe sensor windows";
  }
  return nullptr;
}

void MotorTrial::begin(uint32_t now, bool driverReady) {
  driverReady_ = driverReady;
  outputs_.off(outputs_.context);
  offAt_ = now;
}

const char* MotorTrial::lockReason() const {
  const char* reason = setup_.lockReason();
  if (reason) return reason;
  return driverReady_ ? nullptr : "PWM peripheral did not initialize";
}

bool MotorTrial::inWindow(uint8_t motor, const RangeSample& range) const {
  return range.state == RangeState::Valid && range.mm >= setup_.motors[motor].minMm &&
         range.mm <= setup_.motors[motor].maxMm;
}

const char* MotorTrial::interlock(uint32_t now, uint8_t motor, bool permitted) const {
  if (const char* reason = lockReason()) return reason;
  if (!permitted) return "need a live bidirectional UART heartbeat and Node motion permission";
  if (elapsed(now, offAt_) < config::kMinMotorOffMs) return "minimum motor off time has not elapsed";
  const Reading& a = latest_[motor];
  const Reading& b = previous_[motor];
  if (!inWindow(motor, a.range) || !inWindow(motor, b.range) ||
      elapsed(now, a.at) >= config::kSampleFreshMs ||
      elapsed(a.at, b.at) < config::kSonarQuietMs ||
      elapsed(a.at, b.at) >= config::kSampleFreshMs) return "need two recent in-window readings from the selected sensor";
  if (abs(int(a.range.mm) - int(b.range.mm)) > config::kStationaryJitterMm) return "target is not stationary";
  if (a.range.mm <= setup_.motors[motor].minMm + config::kMovementMm ||
      a.range.mm + config::kMovementMm >= setup_.motors[motor].maxMm) return "target is too close to a window boundary";
  return nullptr;
}

const char* MotorTrial::prepare(uint32_t now, uint8_t motor, bool reverse, uint32_t token, bool permitted) {
  if (state_ == State::Fault) return "fault latched; inspect hardware before clearing";
  if (state_ != State::Idle) return "finish or stop the current trial first";
  if (motor >= 2 || !token) return "invalid motor or token";
  if (const char* reason = interlock(now, motor, permitted)) return reason;
  motor_ = motor;
  reverse_ = reverse;
  token_ = token;
  state_ = State::Confirm;
  stateAt_ = now;
  return nullptr;
}

const char* MotorTrial::confirm(uint32_t now, uint32_t token, bool permitted) {
  if (state_ != State::Confirm) return "prepare a trial first";
  state_ = State::Idle;  // any confirm attempt consumes the request
  if (token != token_ || elapsed(now, stateAt_) >= config::kConfirmMs) return "wrong or expired token";
  if (const char* reason = interlock(now, motor_, permitted)) return reason;
  baselineMm_ = (uint32_t(latest_[motor_].range.mm) + previous_[motor_].range.mm) / 2;
  movementMm_ = 0;
  movedSamples_ = 0;
  results_[motor_][reverse_] = Verdict::Manual;
  state_ = State::Driving;
  stateAt_ = now;
  if (!outputs_.burst(outputs_.context, motor_, reverse_)) {
    fail(now, "hardware burst could not start");
    return fault_;
  }
  return nullptr;
}

void MotorTrial::sample(uint32_t now, uint8_t motor, const RangeSample& sample) {
  previous_[motor] = latest_[motor];
  latest_[motor].range = sample;
  latest_[motor].at = now;
  if (!measuring() || motor != motor_) return;
  if (!inWindow(motor_, sample)) {
    fail(now, "selected sensor invalid or outside the bench window");
    return;
  }
  if (state_ != State::Verify || elapsed(now, offAt_) < config::kMotorSettleMs ||
      elapsed(now, offAt_) >= config::kMovementDeadlineMs) return;

  const int16_t delta = int(sample.mm) - int(baselineMm_);
  if (abs(delta) < config::kMovementMm) {
    movedSamples_ = 0;
    return;
  }
  if (movedSamples_ && elapsed(now, lastObservedAt_) < config::kSonarQuietMs) return;
  lastObservedAt_ = now;
  movedSamples_ = movedSamples_ && ((delta > 0) == (movementMm_ > 0)) ? movedSamples_ + 1 : 1;
  movementMm_ = delta;
  if (movedSamples_ >= 2) {
    state_ = State::Observe;
    stateAt_ = now;
  }
}

void MotorTrial::invalidateSamples() {
  for (uint8_t motor = 0; motor < 2; ++motor) latest_[motor] = previous_[motor] = {};
}

void MotorTrial::tick(uint32_t now, bool permitted) {
  if (measuring() && !permitted) {
    fail(now, "UART or USB interlock lost, or Node inhibited motion");
    return;
  }
  if (measuring() && (!inWindow(motor_, latest_[motor_].range) ||
                      elapsed(now, latest_[motor_].at) >= config::kSampleFreshMs)) {
    fail(now, "selected sensor invalid or stale");
    return;
  }
  if (state_ == State::Driving && elapsed(now, stateAt_) >= config::kMotorBurstMs) {
    if (elapsed(now, stateAt_) > config::kMaxMotorOnMs) {
      fail(now, "control loop missed its motor deadline");
      return;
    }
    outputs_.off(outputs_.context);
    offAt_ = now;
    state_ = State::Verify;
  } else if (state_ == State::Verify && elapsed(now, offAt_) >= config::kMovementDeadlineMs) {
    fail(now, "no sustained displacement; inspect clearance and sensor before retrying");
  } else if ((state_ == State::Confirm && (!permitted || elapsed(now, stateAt_) >= config::kConfirmMs)) ||
             (state_ == State::Observe && elapsed(now, stateAt_) >= config::kObserveMs)) {
    state_ = State::Idle;
  }
}

void MotorTrial::stop(uint32_t now, const char* reason) {
  if (measuring()) {
    fail(now, reason);
  } else {
    outputs_.off(outputs_.context);
    if (state_ != State::Fault) state_ = State::Idle;
  }
}

void MotorTrial::fail(uint32_t now, const char* reason) {
  outputs_.off(outputs_.context);
  offAt_ = now;
  results_[motor_][reverse_] = Verdict::Fail;
  fault_ = reason;
  state_ = State::Fault;
}

bool MotorTrial::recordObservation(uint32_t now, bool ok) {
  if (state_ != State::Observe || elapsed(now, stateAt_) >= config::kObserveMs) return false;
  if (ok) {
    results_[motor_][reverse_] = Verdict::Pass;
    state_ = State::Idle;
  } else {
    fail(now, "operator reported incorrect movement");
  }
  return true;
}

bool MotorTrial::clearFault() {
  if (state_ != State::Fault) return false;
  fault_ = nullptr;
  state_ = State::Idle;
  return true;
}

Verdict MotorTrial::verdict(uint8_t motor) const {
  const auto f = direction(motor, false), r = direction(motor, true);
  if (f == Verdict::Fail || r == Verdict::Fail) return Verdict::Fail;
  if (f == Verdict::Pass && r == Verdict::Pass) return Verdict::Pass;
  return Verdict::Manual;
}
}  // namespace race
