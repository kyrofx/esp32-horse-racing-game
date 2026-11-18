#include "Hardware.h"

#include <driver/gpio.h>
#include <esp_system.h>
#include <soc/soc_caps.h>

namespace race {
bool PulseOutput::begin(uint8_t pin, uint8_t channel, const rmt_item32_t* items, uint16_t count) {
  channel_ = static_cast<rmt_channel_t>(channel);
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  // This is an IDF API; convert the Arduino label exactly once at the boundary.
  const gpio_num_t gpio = static_cast<gpio_num_t>(digitalPinToGPIONumber(pin));
  const rmt_config_t params = RMT_DEFAULT_CONFIG_TX(gpio, channel_);
  // The entire train (and its zero terminator) fits in one block. No ISR refill or loop.
  ready_ = count <= SOC_RMT_MEM_WORDS_PER_CHANNEL && rmt_config(&params) == ESP_OK &&
           rmt_set_source_clk(channel_, RMT_BASECLK_APB) == ESP_OK &&
           rmt_fill_tx_items(channel_, items, count, 0) == ESP_OK;
  if (!ready_) {
    gpio_reset_pin(gpio);  // detach any partially configured RMT output
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  return ready_;
}

bool PulseOutput::start() { return ready_ && rmt_tx_start(channel_, true) == ESP_OK; }
void PulseOutput::off() { if (ready_) rmt_tx_stop(channel_); }

void Sonar::begin() {
  pinMode(board::kEcho, INPUT_PULLDOWN);  // keep a disconnected ECHO from reading as pulses
  echoGpio_ = static_cast<gpio_num_t>(digitalPinToGPIONumber(board::kEcho));
  rmt_item32_t pulses[2]{};
  pulses[0].level0 = 1;
  pulses[0].duration0 = config::kTriggerUs;
  pulses[0].duration1 = 1;  // low, then the all-zero end marker
  ready_ = trigger_.begin(board::kTrig, board::kSonarRmt, pulses, 2);
  if (ready_) attachInterruptArg(board::kEcho, &Sonar::interrupt, this, CHANGE);
}

bool Sonar::start(uint32_t now) {
  if (active_ || pending_ || (sampled_ && elapsed(now, lastShot_) < config::kSonarQuietMs)) return false;
  sampled_ = true;
  lastShot_ = now;
  if (!ready_ || gpio_get_level(echoGpio_)) {
    immediate_ = {ready_ ? RangeState::Timeout : RangeState::HardwareError};
    pending_ = true;
    return true;
  }
  portENTER_CRITICAL(&mux_);
  rising_ = complete_ = false;
  listening_ = true;
  startUs_ = micros();
  portEXIT_CRITICAL(&mux_);
  active_ = trigger_.start();
  if (!active_) {
    portENTER_CRITICAL(&mux_);
    listening_ = false;
    portEXIT_CRITICAL(&mux_);
    immediate_ = {RangeState::HardwareError};
    pending_ = true;
  }
  return true;
}

void IRAM_ATTR Sonar::interrupt(void* self) { static_cast<Sonar*>(self)->edge(); }

void IRAM_ATTR Sonar::edge() {
  const uint32_t stamp = micros();
  const bool high = gpio_get_level(echoGpio_) != 0;
  portENTER_CRITICAL_ISR(&mux_);
  if (listening_) {
    if (high && !rising_) {
      riseUs_ = stamp;
      rising_ = true;
    } else if (!high && rising_) {
      fallUs_ = stamp;
      complete_ = true;
      listening_ = false;
    }
  }
  portEXIT_CRITICAL_ISR(&mux_);
}

bool Sonar::poll(RangeSample& out) {
  if (pending_) {
    pending_ = false;
    out = immediate_;
    return true;
  }
  if (!active_) return false;
  const bool timedOut = elapsed(micros(), startUs_) >= config::kEchoTimeoutUs;
  portENTER_CRITICAL(&mux_);
  const bool complete = complete_;
  const uint32_t fall = fallUs_, rise = riseUs_;
  if (complete || timedOut) listening_ = false;
  portEXIT_CRITICAL(&mux_);
  if (!complete && !timedOut) return false;
  active_ = false;
  if (complete && elapsed(fall, startUs_) <= config::kEchoTimeoutUs && elapsed(fall, rise) <= config::kEchoTimeoutUs) {
    out = measureEcho(uint16_t(elapsed(fall, rise)));
  } else {
    out = {RangeState::Timeout};
  }
  return true;
}

void Uart::begin() {
  Serial1.setTxBufferSize(board::kUartTxBuffer);
  Serial1.begin(config::kLinkBaud, SERIAL_8N1, board::kUartRx, board::kUartTx);
  link.begin(esp_random());
}

bool Uart::write(void*, const uint8_t* bytes, size_t size) {
  return Serial1.availableForWrite() >= int(size) && Serial1.write(bytes, size) == size;
}

#if defined(ARDUINO_NANO_ESP32)
bool MotorDriver::begin() {
  for (const auto& pair : board::kDirection) {
    for (uint8_t pin : pair) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
    }
  }
  if (!board::kBench.enablePinsWired) return false;
  for (uint8_t pin : board::kEnables) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  if (board::kBench.lockReason()) return false;
  constexpr size_t kCycles = config::kMotorBurstMs * 1000UL / config::kPwmPeriodUs;
  static_assert(config::kMaxMotorOnMs * 1000UL / config::kPwmPeriodUs + 1 <= SOC_RMT_MEM_WORDS_PER_CHANNEL,
                "PWM train plus terminator must fit in hardware RAM");
  rmt_item32_t train[kCycles + 1]{};
  for (size_t i = 0; i < kCycles; ++i) {
    train[i].level0 = 1;
    train[i].duration0 = config::kPwmPeriodUs * config::kMotorDutyPercent / 100;
    train[i].level1 = 0;
    train[i].duration1 = config::kPwmPeriodUs - train[i].duration0;
  }
  const bool a = enables_[0].begin(board::kEnables[0], board::kMotorRmt[0], train, kCycles + 1);
  const bool b = enables_[1].begin(board::kEnables[1], board::kMotorRmt[1], train, kCycles + 1);
  return ready_ = a && b;
}

bool MotorDriver::start(void* self, uint8_t motor, bool reverse) {
  auto& driver = *static_cast<MotorDriver*>(self);
  if (!driver.ready_ || motor >= 2) return false;
  driver.off();
  digitalWrite(board::kDirection[motor][reverse ? 1 : 0], HIGH);
  if (driver.enables_[motor].start()) return true;
  driver.off();
  return false;
}

void MotorDriver::stop(void* self) { static_cast<MotorDriver*>(self)->off(); }

void MotorDriver::off() {
  for (auto& enable : enables_) enable.off();
  for (const auto& pair : board::kDirection) {
    for (uint8_t pin : pair) digitalWrite(pin, LOW);
  }
}
#endif
}  // namespace race
