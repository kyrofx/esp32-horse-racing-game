#pragma once

#include <Arduino.h>
#include <driver/rmt.h>

#include "Board.h"
#include "Config.h"
#include "Link.h"

namespace race {
class PulseOutput {
 public:
  bool begin(uint8_t pin, uint8_t channel, const rmt_item32_t* items, uint16_t count);
  bool start();
  void off();

 private:
  rmt_channel_t channel_ = RMT_CHANNEL_0;
  bool ready_ = false;
};

class Sonar {
 public:
  void begin();
  bool start(uint32_t now);
  bool poll(RangeSample& out);

 private:
  static void IRAM_ATTR interrupt(void* self);
  void IRAM_ATTR edge();

  PulseOutput trigger_;
  gpio_num_t echoGpio_ = GPIO_NUM_NC;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  bool ready_ = false, active_ = false, pending_ = false, sampled_ = false;
  RangeSample immediate_;
  uint32_t lastShot_ = 0, startUs_ = 0;
  // Written by the GPIO interrupt or accessed with mux_ held.
  volatile bool listening_ = false, rising_ = false, complete_ = false;
  volatile uint32_t riseUs_ = 0, fallUs_ = 0;
};

class Uart {
 public:
  explicit Uart(wire::Role role) : link(role, &Uart::write, this) {}
  void begin();
  template <typename Handler>
  void poll(Handler handler) {
    for (uint8_t i = 0; i < board::kUartReadBudget && Serial1.available(); ++i) {
      wire::Frame frame{};
      const int byte = Serial1.read();
      if (byte >= 0 && link.receive(uint8_t(byte), millis(), frame)) handler(frame);
    }
  }

  wire::Link link;

 private:
  static bool write(void*, const uint8_t* bytes, size_t size);
};

#if defined(ARDUINO_NANO_ESP32)
class MotorDriver {
 public:
  bool begin();
  MotorTrial::Outputs outputs() { return {&MotorDriver::start, &MotorDriver::stop, this}; }

 private:
  static bool start(void* self, uint8_t motor, bool reverse);
  static void stop(void* self);
  void off();

  PulseOutput enables_[2];
  bool ready_ = false;
};
#endif
}  // namespace race
