#pragma once

#include "Config.h"

namespace race {
// A switch held at startup does not satisfy the test. Observe a fresh press and release.
class ButtonTracker {
 public:
  void reset(bool pressed, uint32_t now) {
    raw_ = stable_ = pressed;
    changedAt_ = now;
    sawPress_ = passed_ = false;
  }

  bool update(bool pressed, uint32_t now) {
    if (pressed != raw_) {
      raw_ = pressed;
      changedAt_ = now;
    }
    if (raw_ == stable_ || elapsed(now, changedAt_) < config::kButtonDebounceMs) return false;
    stable_ = raw_;
    if (stable_) sawPress_ = true;
    else if (sawPress_) passed_ = true;
    return true;
  }

  bool pressed() const { return stable_; }
  bool passed() const { return passed_; }

 private:
  bool raw_ = false, stable_ = false, sawPress_ = false, passed_ = false;
  uint32_t changedAt_ = 0;
};
}  // namespace race
