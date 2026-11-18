#pragma once

#include <stdint.h>

namespace race {

struct ButtonMapping {
  static constexpr uint8_t kUnassigned = 0xff;
  uint8_t inputForLed[8] = {kUnassigned, kUnassigned, kUnassigned, kUnassigned,
                            kUnassigned, kUnassigned, kUnassigned, kUnassigned};

  bool valid() const;
  uint8_t ledMask(uint8_t player, uint8_t pressed) const;
};

class ButtonPairing {
 public:
  enum class Event : uint8_t { None, Assigned, Multiple, WrongPlayer, Reused };

  void start();
  void cancel();
  Event update(uint8_t pressed, bool targetReady);

  bool active() const { return step_ < 8; }
  uint8_t step() const { return step_; }
  const ButtonMapping& draft() const { return draft_; }

 private:
  void waitForRelease();

  ButtonMapping draft_;
  uint8_t step_ = 8;
  uint8_t chosen_ = ButtonMapping::kUnassigned;
  bool armed_ = false;
};

}  // namespace race
