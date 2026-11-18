#pragma once

#include <Arduino.h>

#include "MotorTrial.h"

namespace race {
namespace board {
// Nano pin constants are header labels. Arduino APIs apply the variant's pin remap.
#if defined(ARDUINO_NANO_ESP32)
constexpr uint8_t kPlayer = 2;
constexpr const char* kName = "Arduino Nano ESP32";
constexpr uint8_t kLeds[] = {D2, D3, D4, D5};
constexpr uint8_t kOledAddress = 0x3D;
constexpr bool kRotateOled180 = false;
constexpr uint8_t kOledHeight = 64;  // set to 32 for a 128x32 panel
constexpr uint8_t kSda = A4, kScl = A5, kTrig = A2, kEcho = A3;
constexpr uint8_t kUartTx = D8, kUartRx = D9;
constexpr uint8_t kDirection[2][2] = {{D6, D7}, {D10, D11}};
constexpr uint8_t kEnables[] = {D12, A0};  // optional; jumpers must be removed
constexpr uint8_t kMotorRmt[] = {0, 1};
constexpr uint8_t kSonarRmt = 2;

// Fill in from the actual bench setup. Zero/false/empty leaves powered testing locked.
constexpr BenchSetup kBench = {
    false,  // both enable jumpers removed; D12 -> ENA and A0 -> ENB with pull-downs
    0,      // measured motor supply at L298N, mV
    0,      // external supply current limit SETTING, mA (not measured motor current)
    {
        {"", 0, 0, 0},  // motor 1: exact model, rated mV, safe target window min/max mm
        {"", 0, 0, 0},  // motor 2: exact model, rated mV, safe target window min/max mm
    }};
#elif defined(ARDUINO_NodeMCU_32S)
constexpr uint8_t kPlayer = 1;
constexpr const char* kName = "NodeMCU-32S";
constexpr uint8_t kLeds[] = {25, 26, 27, 32};
constexpr uint8_t kOledAddress = 0x3C;
constexpr bool kRotateOled180 = true;
constexpr uint8_t kOledHeight = 64;  // set to 32 for a 128x32 panel
constexpr uint8_t kButtons[] = {35, 34, 36, 39, 16, 17, 18, 19};
constexpr uint8_t buttonMode(uint8_t gpio) {
  return gpio >= 34 ? INPUT : INPUT_PULLUP;  // GPIO34–39 use external pull-ups.
}
constexpr uint8_t kSda = 21, kScl = 22, kTrig = 23, kEcho = 33;
constexpr uint8_t kUartTx = 13, kUartRx = 14;
constexpr uint8_t kSonarRmt = 0;
#else
#error Select the NodeMCU-32S or Arduino Nano ESP32 PlatformIO environment.
#endif

constexpr uint8_t kOledWidth = 128;
constexpr uint8_t kLedOn = HIGH, kLedOff = LOW;
constexpr uint8_t kButtonPressed = LOW;
constexpr size_t kUartTxBuffer = 256;
constexpr uint8_t kUartReadBudget = 96;
static_assert(kOledHeight == 32 || kOledHeight == 64, "Use a 128x32 or 128x64 SSD1306");
}  // namespace board
}  // namespace race
