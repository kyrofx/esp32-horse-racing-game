#include "ButtonPairing.h"

namespace race {

bool ButtonMapping::valid() const {
  for (uint8_t player = 0; player < 2; ++player) {
    uint8_t used = 0;
    for (uint8_t led = 0; led < 4; ++led) {
      const uint8_t input = inputForLed[player * 4 + led];
      if (input >= 4 || (used & (1u << input))) return false;
      used |= 1u << input;
    }
  }
  return true;
}

uint8_t ButtonMapping::ledMask(uint8_t player, uint8_t pressed) const {
  uint8_t mask = 0;
  for (uint8_t led = 0; led < 4; ++led) {
    const uint8_t input = inputForLed[player * 4 + led];
    if (input < 4 && (pressed & (1u << input))) mask |= 1u << led;
  }
  return mask;
}

void ButtonPairing::waitForRelease() {
  armed_ = false;
  chosen_ = ButtonMapping::kUnassigned;
}

void ButtonPairing::start() {
  draft_ = {};
  step_ = 0;
  waitForRelease();
}

void ButtonPairing::cancel() {
  step_ = 8;
  waitForRelease();
}

ButtonPairing::Event ButtonPairing::update(uint8_t pressed, bool targetReady) {
  if (!active()) return Event::None;
  if (!targetReady) {
    waitForRelease();
    return Event::None;
  }
  if (!armed_) {
    armed_ = pressed == 0;
    return Event::None;
  }

  const uint8_t bankStart = (step_ / 4) * 4;
  if (chosen_ != ButtonMapping::kUnassigned) {
    if (pressed == (1u << (bankStart + chosen_))) return Event::None;
    if (pressed != 0) {
      waitForRelease();
      return Event::Multiple;
    }

    draft_.inputForLed[step_++] = chosen_;
    waitForRelease();
    return Event::Assigned;
  }

  if (pressed == 0) return Event::None;
  if (pressed & (pressed - 1)) {
    waitForRelease();
    return Event::Multiple;
  }
  const uint8_t bankInputs = (pressed >> bankStart) & 0x0f;
  if (bankInputs == 0) {
    waitForRelease();
    return Event::WrongPlayer;
  }
  uint8_t input = 0;
  while ((bankInputs & (1u << input)) == 0) ++input;
  for (uint8_t prior = bankStart; prior < step_; ++prior) {
    if (draft_.inputForLed[prior] == input) {
      waitForRelease();
      return Event::Reused;
    }
  }
  chosen_ = input;
  return Event::None;
}

}  // namespace race
