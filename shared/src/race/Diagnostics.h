#pragma once

#include <Arduino.h>
#include <string.h>

#include "Board.h"
#include "Config.h"
#include "Link.h"

namespace race {
class Console {
 public:
  struct Command { char* words[4]; uint8_t count; };

  void begin();
  bool read(Command& out);
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void flushSome();
  uint32_t dropped() const { return dropped_; }

 private:
  static constexpr size_t kRows = 32, kColumns = 160, kCommandBytes = 80;
  char rows_[kRows][kColumns]{};
  char line_[kCommandBytes]{};
  uint8_t head_ = 0, queued_ = 0, sent_ = 0, inputSize_ = 0;
  bool overlong_ = false;
  uint32_t dropped_ = 0;
};

inline bool is(const Console::Command& cmd, uint8_t i, const char* word) {
  return cmd.count <= 4 && i < cmd.count && strcmp(cmd.words[i], word) == 0;
}
inline bool confirmation(const Console::Command& cmd, uint8_t i) {
  return is(cmd, i, "ok") || is(cmd, i, "fail");
}

struct RangeLog {
  RangeSample last;
  uint32_t receivedAt = 0;
  uint32_t counts[5]{};
  void sample(RangeSample reading, uint32_t now);
  void print(Console& out, uint8_t player, uint32_t now) const;
};

class OledTest {
 public:
  void start();
  void tick(Console& out);
  bool confirm(bool ok);
  void motorScreen(const char (&rows)[4][22], Console& out);
  bool busy() const { return step_ != Step::Idle; }
  Verdict bus() const { return bus_; }
  Verdict display() const { return display_; }

 private:
  enum class Step : uint8_t { Idle, Probe, Setup, Draw, Show };
  uint8_t send(uint8_t control, const uint8_t* bytes, size_t size);
  void failed(Console& out, uint8_t error);

  Step step_ = Step::Idle;
  bool initialized_ = false, drawable_ = false;
  uint16_t offset_ = 0;
  char screenRows_[8][22]{};
  uint8_t screenDirty_ = 0xff, screenChunk_ = 0, screenPage_ = 0xff;
  Verdict bus_ = Verdict::Manual, display_ = Verdict::Manual;
};

class LedTest {
 public:
  void begin();
  void start(uint32_t now, Console& out);
  void tick(uint32_t now, Console& out);
  void setLedMask(uint8_t mask, Console& out);
  void reportLedMask(Console& out) const;
  bool confirm(uint8_t index, bool ok);
  bool busy() const { return active_ < 4; }
  uint8_t appliedMask() const { return busy() ? config::kLedSweepActive : requestedMask_; }
  Verdict verdict() const;

 private:
  void drive(uint8_t mask);
  void show(uint32_t now, Console& out);
  uint8_t active_ = 4, tested_ = 0, seen_ = 0, failed_ = 0;
  uint8_t requestedMask_ = 0;
  uint32_t at_ = 0;
};

class Diagnostics {
 public:
  void begin();
  void start(uint32_t now);
  void tick(uint32_t now);
  bool command(const Console::Command& cmd);
  void status();
  void linkStatus(const wire::Link& link, uint32_t now);
  bool busy() const { return leds.busy() || oled.busy(); }

  Console console;
  OledTest oled;
  LedTest leds;
  bool stream = false;
};
}  // namespace race
