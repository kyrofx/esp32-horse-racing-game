#pragma once

#include "Config.h"

namespace race {
struct MotorSpec {
  const char* model;
  uint16_t ratedMv, minMm, maxMm;
};

struct BenchSetup {
  bool enablePinsWired;
  uint16_t measuredSupplyMv, supplyLimitSettingMa;
  MotorSpec motors[2];
  const char* lockReason() const;
};

class MotorTrial {
 public:
  enum class State : uint8_t { Idle, Confirm, Driving, Verify, Observe, Fault };
  struct Outputs {
    bool (*burst)(void*, uint8_t motor, bool reverse);
    void (*off)(void*);
    void* context;
  };

  MotorTrial(const BenchSetup& setup, Outputs outputs) : setup_(setup), outputs_(outputs) {}
  void begin(uint32_t now, bool driverReady);
  const char* prepare(uint32_t now, uint8_t motor, bool reverse, uint32_t token, bool permitted);
  const char* confirm(uint32_t now, uint32_t token, bool permitted);
  void sample(uint32_t now, uint8_t motor, const RangeSample& sample);
  void invalidateSamples();
  void tick(uint32_t now, bool permitted);
  void stop(uint32_t now, const char* reason);
  bool recordObservation(uint32_t now, bool ok);
  bool clearFault();

  State state() const { return state_; }
  const char* fault() const { return fault_; }
  const char* lockReason() const;
  uint8_t motor() const { return motor_; }
  bool reverse() const { return reverse_; }
  int16_t movementMm() const { return movementMm_; }
  Verdict direction(uint8_t motor, bool reverse) const { return results_[motor][reverse]; }
  Verdict verdict(uint8_t motor) const;

 private:
  struct Reading { RangeSample range; uint32_t at = 0; };
  const char* interlock(uint32_t now, uint8_t motor, bool permitted) const;
  bool inWindow(uint8_t motor, const RangeSample& range) const;
  bool measuring() const { return state_ == State::Driving || state_ == State::Verify; }
  void fail(uint32_t now, const char* reason);

  const BenchSetup& setup_;
  Outputs outputs_;
  State state_ = State::Idle;
  bool driverReady_ = false, reverse_ = false;
  uint8_t motor_ = 0, movedSamples_ = 0;
  uint16_t baselineMm_ = 0;
  int16_t movementMm_ = 0;
  uint32_t token_ = 0, stateAt_ = 0, offAt_ = 0, lastObservedAt_ = 0;
  const char* fault_ = nullptr;
  Reading latest_[2]{}, previous_[2]{};
  Verdict results_[2][2]{};
};
}  // namespace race
