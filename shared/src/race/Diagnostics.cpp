#include "Diagnostics.h"

#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace race {
namespace {
constexpr uint8_t kChunk = 32;

uint8_t glyph(char c, uint8_t col) {
  static const uint8_t p[] = {0x7f, 0x09, 0x09, 0x09, 0x06};
  static const uint8_t one[] = {0, 0x42, 0x7f, 0x40, 0};
  static const uint8_t two[] = {0x62, 0x51, 0x49, 0x49, 0x46};
  static const uint8_t t[] = {1, 1, 0x7f, 1, 1};
  static const uint8_t e[] = {0x7f, 0x49, 0x49, 0x49, 0x41};
  static const uint8_t s[] = {0x26, 0x49, 0x49, 0x49, 0x32};
  switch (c) {
    case 'P': return p[col];
    case '1': return one[col];
    case '2': return two[col];
    case 'T': return t[col];
    case 'E': return e[col];
    case 'S': return s[col];
    default: break;
  }
  static constexpr char extra[] = "03456789%ADGIKLMNORUWY";
  static const uint8_t font[][5] = {
    {62,81,73,69,62},{33,65,69,75,49},{24,20,18,127,16},{39,69,69,69,57},
    {60,74,73,73,48},{1,113,9,5,3},{54,73,73,73,54},{6,73,73,41,30},
    {99,19,8,100,99},{126,17,17,17,126},{127,65,65,34,28},{62,65,73,73,122},
    {0,65,127,65,0},{127,8,20,34,65},{127,64,64,64,64},{127,2,12,2,127},
    {127,4,8,16,127},{62,65,65,65,62},{127,9,25,41,70},{63,64,64,64,63},
    {63,64,56,64,63},{7,8,112,8,7}
  };
  static_assert(sizeof(extra) - 1 == sizeof(font)/sizeof(font[0]), "missing glyph");
  for (size_t i = 0; i < sizeof(extra)-1; ++i) {
    if (c == extra[i]) return font[i][col];
  }
  return 0;
}

uint8_t pattern(uint16_t offset) {
  const uint8_t x = offset % board::kOledWidth, page = offset / board::kOledWidth;
  if (x == 0 || x == board::kOledWidth - 1) return 0xff;
  if (page == 0) return 0x01;
  if (page == board::kOledHeight / 8 - 1) return (x / 4) % 2 ? 0x8f : 0x80;
  constexpr uint8_t kTitlePage = board::kOledHeight / 16 - 1;
  constexpr uint8_t kTitleWidth = 7 * 12;
  const int col = x - (board::kOledWidth - kTitleWidth) / 2;
  if ((page != kTitlePage && page != kTitlePage + 1) || col < 0 || col >= kTitleWidth || col % 12 >= 10) return 0;
  const char title[] = {'P', char('0' + board::kPlayer), ' ', 'T', 'E', 'S', 'T'};
  const uint8_t dots = glyph(title[col / 12], (col % 12) / 2);
  uint16_t doubled = 0;
  for (uint8_t bit = 0; bit < 7; ++bit) if (dots & (1u << bit)) doubled |= 3u << (2 * bit);
  return uint8_t(doubled >> (8 * (page - kTitlePage)));
}
}  // namespace

void Console::begin() {
#if defined(ARDUINO_NANO_ESP32)
  Serial.setTxTimeoutMs(0);
#else
  Serial.setTxBufferSize(256);
#endif
  Serial.begin(config::kConsoleBaud);
}

bool Console::read(Command& out) {
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '!') {
      inputSize_ = 0;
      overlong_ = false;
      line_[0] = '!';
      line_[1] = '\0';
      out.words[0] = line_;
      out.count = 1;
      return true;
    }
    if (c == '\r' || c == '\n') {
      line_[inputSize_] = '\0';
      inputSize_ = 0;
      if (overlong_) {
        overlong_ = false;
        log("Malformed or too-long command discarded.");
      } else {
        out.count = 0;
        char* save = nullptr;
        for (char* word = strtok_r(line_, " \t", &save); word; word = strtok_r(nullptr, " \t", &save)) {
          if (out.count == 4) { ++out.count; break; }
          out.words[out.count++] = word;
        }
        if (out.count) return true;
      }
    } else if (!overlong_) {
      if (c == '\b' || c == 127) {
        if (inputSize_) --inputSize_;
      } else if (c == '\t' || (c >= 32 && c <= 126)) {
        if (inputSize_ < sizeof(line_) - 1) line_[inputSize_++] = char(c);
        else overlong_ = true;
      } else {
        // Reject malformed commands rather than ignoring bytes inside a confirmation token.
        overlong_ = true;
      }
    }
  }
  return false;
}

void Console::log(const char* fmt, ...) {
  if (queued_ == kRows) { ++dropped_; return; }
  char* const dest = rows_[(head_ + queued_) % kRows];
  va_list args;
  va_start(args, fmt);
  vsnprintf(dest, kColumns - 2, fmt, args);
  va_end(args);
  const size_t len = strlen(dest);
  dest[len] = '\r';
  dest[len + 1] = '\n';
  dest[len + 2] = '\0';
  ++queued_;
}

void Console::flushSome() {
  if (!queued_) return;
  const int room = Serial.availableForWrite();
  if (room <= 0) return;
  const char* const line = rows_[head_];
  const size_t left = strlen(line + sent_);
  size_t count = left < size_t(room) ? left : size_t(room);
  if (count > 64) count = 64;
  const size_t written = Serial.write(reinterpret_cast<const uint8_t*>(line + sent_), count);
  if (written == left) {
    head_ = (head_ + 1) % kRows;
    --queued_;
    sent_ = 0;
  } else {
    sent_ += written;
  }
}

void RangeLog::sample(RangeSample reading, uint32_t now) {
  last = reading;
  receivedAt = now;
  ++counts[uint8_t(last.state)];
}

void RangeLog::print(Console& out, uint8_t player, uint32_t now) const {
  const char* status = "NOT SAMPLED";
  Verdict verdict = Verdict::Manual;
  if (last.state != RangeState::None) {
    verdict = Verdict::Fail;
    if (elapsed(now, receivedAt) >= config::kSampleFreshMs) status = "STALE";
    else if (last.state == RangeState::Valid) { verdict = Verdict::Pass; status = "VALID"; }
    else if (last.state == RangeState::OutOfRange) status = "OUT OF RANGE";
    else if (last.state == RangeState::Timeout) status = "TIMEOUT";
    else status = "TRIGGER ERROR";
  }
  if (last.echoUs && (last.state == RangeState::Valid || last.state == RangeState::OutOfRange)) {
    out.log("P%u sonar %s: %s %u mm (%u us); valid=%lu range=%lu timeout=%lu error=%lu", player, label(verdict), status,
            last.mm, last.echoUs, (unsigned long)counts[1], (unsigned long)counts[2], (unsigned long)counts[3],
            (unsigned long)counts[4]);
  } else {
    out.log("P%u sonar %s: %s; valid=%lu range=%lu timeout=%lu error=%lu", player, label(verdict), status,
            (unsigned long)counts[1], (unsigned long)counts[2], (unsigned long)counts[3], (unsigned long)counts[4]);
  }
}

void OledTest::start() {
  screenDirty_ = 0xff;
  screenPage_ = 0xff;
  step_ = Step::Probe;
  offset_ = 0;
  drawable_ = false;
  bus_ = display_ = Verdict::Manual;
}

uint8_t OledTest::send(uint8_t control, const uint8_t* bytes, size_t size) {
  Wire.beginTransmission(board::kOledAddress);
  Wire.write(control);
  if (size) Wire.write(bytes, size);
  return Wire.endTransmission();
}

void OledTest::failed(Console& out, uint8_t error) {
  bus_ = Verdict::Fail;
  step_ = Step::Idle;
  out.log("OLED I2C FAIL at 0x%02X (Wire code %u). Check power, pull-ups, SDA/SCL and address.", board::kOledAddress, error);
}

void OledTest::tick(Console& out) {
  switch (step_) {
    case Step::Idle: return;
    case Step::Probe: {
      if (!initialized_) initialized_ = Wire.begin(board::kSda, board::kScl, config::kI2cHz);
      if (!initialized_) { failed(out, 4); return; }
      Wire.setTimeOut(config::kI2cTimeoutMs);
      Wire.beginTransmission(board::kOledAddress);
      const uint8_t code = Wire.endTransmission();
      if (code) { failed(out, code); return; }
      bus_ = Verdict::Pass;
      out.log("OLED I2C PASS: address 0x%02X acknowledged.", board::kOledAddress);
      step_ = Step::Setup;
      return;
    }
    case Step::Setup: {
      const uint8_t cmds[] = {
          0xae, 0xd5, 0x80, 0xa8, board::kOledHeight - 1, 0xd3, 0x00, 0x40,
          0x8d, 0x14, 0x20, 0x00, board::kRotateOled180 ? 0xa0 : 0xa1,
          board::kRotateOled180 ? 0xc0 : 0xc8, 0xda, board::kOledHeight == 64 ? 0x12 : 0x02,
          0x81, 0x8f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e,
          0x21, 0, board::kOledWidth - 1, 0x22, 0, board::kOledHeight / 8 - 1};
      const uint8_t code = send(0x00, cmds, sizeof(cmds));
      if (code) { failed(out, code); return; }
      step_ = Step::Draw;
      return;
    }
    case Step::Draw: {
      uint8_t chunk[kChunk];
      for (uint8_t i = 0; i < sizeof(chunk); ++i) chunk[i] = pattern(offset_ + i);
      const uint8_t code = send(0x40, chunk, sizeof(chunk));
      if (code) { failed(out, code); return; }
      offset_ += sizeof(chunk);
      if (offset_ == board::kOledWidth * board::kOledHeight / 8) step_ = Step::Show;
      return;
    }
    case Step::Show: {
      const uint8_t on = 0xaf;
      const uint8_t code = send(0x00, &on, 1);
      if (code) { failed(out, code); return; }
      drawable_ = true;
      step_ = Step::Idle;
      out.log("OLED display MANUAL CHECK: look for P%u TEST, border and bottom bars; enter oled ok or oled fail.", board::kPlayer);
      return;
    }
  }
}

void OledTest::motorScreen(const char (&rows)[4][22], Console& out) {
  if (busy() || bus_ != Verdict::Pass) return;
  drawable_ = false;
  constexpr uint8_t pages = board::kOledHeight / 8;
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t page = i * (pages / 4);
    if (strcmp(rows[i], screenRows_[page]) != 0) {
      strcpy(screenRows_[page], rows[i]);
      screenDirty_ |= 1u << page;
      if (page == screenPage_) screenChunk_ = 0;
    }
  }
  if (!screenDirty_) return;
  if (screenPage_ >= pages) {
    for (screenPage_ = 0; screenPage_ < pages; ++screenPage_)
      if (screenDirty_ & (1u << screenPage_)) break;
    if (screenPage_ == pages) { screenDirty_ = 0; return; }
    screenChunk_ = 0;
  }
  const uint8_t x = screenChunk_ * kChunk;
  const uint8_t region[] = {0x21, x, uint8_t(x + kChunk - 1), 0x22, screenPage_, screenPage_};
  uint8_t data[kChunk]{};
  const char* line = screenRows_[screenPage_];
  for (uint8_t i = 0; i < kChunk; ++i) {
    const uint8_t pos = x + i;
    if (pos / 6 < strlen(line) && pos % 6 < 5) data[i] = glyph(line[pos / 6], pos % 6);
  }
  uint8_t error = send(0, region, sizeof(region));
  if (!error) error = send(0x40, data, sizeof(data));
  if (error) { failed(out, error); return; }
  if (++screenChunk_ == board::kOledWidth / kChunk) {
    screenDirty_ &= ~(1u << screenPage_);
    screenPage_ = 0xff;
  }
}

bool OledTest::confirm(bool ok) {
  if (!drawable_) return false;
  display_ = ok ? Verdict::Pass : Verdict::Fail;
  return true;
}

void LedTest::begin() {
  for (auto pin : board::kLeds) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, board::kLedOff);
  }
}

void LedTest::start(uint32_t now, Console& out) {
  active_ = tested_ = seen_ = failed_ = 0;
  show(now, out);
}

void LedTest::drive(uint8_t mask) {
  for (uint8_t i = 0; i < 4; ++i) {
    const bool on = (mask & (1u << i)) != 0;
    digitalWrite(board::kLeds[i], on ? board::kLedOn : board::kLedOff);
  }
}

void LedTest::setLedMask(uint8_t mask, Console& out) {
  if (requestedMask_ == mask) return;
  requestedMask_ = mask;
  if (!busy()) {
    drive(requestedMask_);
    reportLedMask(out);
  }
}

void LedTest::reportLedMask(Console& out) const {
  if (busy()) return;
  char driven[] = "----";
  for (uint8_t i = 0; i < 4; ++i) {
    if (requestedMask_ & (1u << i)) driven[i] = '1' + i;
  }
  out.log("P%u LEDs commanded [%s]", board::kPlayer, driven);
}

void LedTest::show(uint32_t now, Console& out) {
  at_ = now;
  if (busy()) {
    drive(1u << active_);
    out.log("P%u LED %u commanded on; watch the physical LED.", board::kPlayer, active_ + 1);
  } else {
    drive(requestedMask_);
    out.log("P%u LEDs MANUAL CHECK: enter led <1-4> ok or led <1-4> fail for each step.", board::kPlayer);
    reportLedMask(out);
  }
}

void LedTest::tick(uint32_t now, Console& out) {
  if (!busy() || elapsed(now, at_) < config::kLedStepMs) return;
  tested_ |= 1u << active_++;
  show(now, out);
}

bool LedTest::confirm(uint8_t index, bool ok) {
  const uint8_t bit = 1u << index;
  if (!(tested_ & bit)) return false;
  if (ok) { seen_ |= bit; failed_ &= ~bit; }
  else { failed_ |= bit; seen_ &= ~bit; }
  return true;
}

Verdict LedTest::verdict() const {
  if (failed_) return Verdict::Fail;
  return seen_ == 0x0f ? Verdict::Pass : Verdict::Manual;
}

void Diagnostics::begin() {
  console.begin();
  leds.begin();
  console.log("%s | P%u | TEST MODE | enter help or test", board::kName, board::kPlayer);
}

void Diagnostics::start(uint32_t now) {
  oled.start();
  leds.start(now, console);
  stream = true;
}

void Diagnostics::tick(uint32_t now) {
  leds.tick(now, console);
  oled.tick(console);
}

bool Diagnostics::command(const Console::Command& cmd) {
  if (cmd.count == 2 && is(cmd, 0, "oled") && confirmation(cmd, 1)) {
    if (oled.confirm(is(cmd, 1, "ok"))) console.log("P%u OLED display %s (operator).", board::kPlayer, label(oled.display()));
    else console.log("Run test and wait for the OLED test screen first.");
    return true;
  }
  if (cmd.count == 3 && is(cmd, 0, "led") && confirmation(cmd, 2) &&
      cmd.words[1][0] >= '1' && cmd.words[1][0] <= '4' && !cmd.words[1][1]) {
    const uint8_t led = cmd.words[1][0] - '1';
    if (leds.confirm(led, is(cmd, 2, "ok"))) console.log("P%u LED %u %s (operator). LED bank: %s.", board::kPlayer,
                                                      led + 1, is(cmd, 2, "ok") ? "PASS" : "FAIL", label(leds.verdict()));
    else console.log("Wait for LED %u to finish its test step.", led + 1);
    return true;
  }
  if (cmd.count == 2 && is(cmd, 0, "stream") && (is(cmd, 1, "on") || is(cmd, 1, "off"))) {
    stream = is(cmd, 1, "on");
    console.log("Periodic sonar report %s.", stream ? "on" : "off");
    return true;
  }
  return false;
}

void Diagnostics::status() {
  console.log("P%u OLED I2C: %s | screen: %s | LEDs: %s", board::kPlayer, label(oled.bus()), label(oled.display()), label(leds.verdict()));
  leds.reportLedMask(console);
  if (console.dropped()) console.log("Console messages dropped while host was busy: %lu. Re-run status.", (unsigned long)console.dropped());
}

void Diagnostics::linkStatus(const wire::Link& link, uint32_t now) {
  console.log("UART heartbeat: %s | frame/CRC errors=%lu | self-echoes=%lu | wrong role=%lu",
              label(link.online(now) ? Verdict::Pass : Verdict::Fail), (unsigned long)link.errors(),
              (unsigned long)link.selfEchoes(), (unsigned long)link.wrongRoleFrames());
}
}  // namespace race
