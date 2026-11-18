#include <esp_system.h>

#include <race/Diagnostics.h>
#include <race/Hardware.h>

namespace {
using namespace race;
using wire::Kind;
using State = MotorTrial::State;

bool parseToken(const char* word, uint32_t& out) {
  if (strlen(word) != 8) return false;
  out = 0;
  while (*word) {
    const char c = *word++;
    uint8_t digit;
    if (c >= '0' && c <= '9') digit = c - '0';
    else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
    else return false;
    out = (out << 4) | digit;
  }
  return true;
}

const char* stateName(State state) {
  switch (state) {
    case State::Confirm: return "awaiting one-shot confirmation";
    case State::Driving: return "burst in progress";
    case State::Verify: return "power off; checking displacement";
    case State::Observe: return "power off; awaiting manual observation";
    case State::Fault: return "FAULT LATCHED";
    default: return "disarmed";
  }
}

class Controller {
 public:
  void begin() {
    const bool driverReady = driver_.begin();  // make motor outputs safe before other peripherals
    trial_.begin(millis(), driverReady);
    buttonReady_ = !board::kBench.enablePinsWired;
    diag_.begin();
    uart_.begin();
    sonar_.begin();
    if (const char* reason = trial_.lockReason()) diag_.console.log("Motor tests locked: %s.", reason);
    else diag_.console.log("Motor tests configured. Disarmed; a fresh confirmation is required for every burst.");
  }

  void tick() {
    const uint32_t stamp = millis();
    if (buttonArmed_ && (!uart_.link.online(stamp) || elapsed(stamp, buttonRxAt_) >= config::kLinkTimeoutMs))
      disarmButton(stamp);
    if (buttonActive_ && elapsed(stamp, buttonStarted_) >= config::kButtonDriveMs) endButton(stamp);
    serviceTrial();
    uart_.poll([this](const wire::Frame& frame) { receive(frame, millis()); });
    peerChanged(millis());
    serviceTrial();
    const uint32_t nowLeds = millis();
    if (!uart_.link.online(nowLeds) || elapsed(nowLeds, ledsReceivedAt_) >= config::kLinkTimeoutMs) remoteLedMask_ = 0;
    diag_.leds.setLedMask(remoteLedMask_, diag_.console);
    RangeSample sample;
    if (sonar_.poll(sample)) {
      const uint32_t now = millis();
      ranges_[1].sample(sample, now);
      trial_.sample(now, 1, sample);
      uint8_t data[7];
      wire::packSample(sample, data, shotSeq_);
      uart_.link.send(Kind::Sample, data, sizeof(data));
    }

    Console::Command cmd{};
    if (diag_.console.read(cmd)) command(cmd, millis());

    const uint32_t now = millis();
    trial_.tick(now, motionPermitted(now));
    uart_.link.tick(now);
    reportChanges(now);
    if (diag_.stream && elapsed(now, reportAt_) >= config::kConsoleReportMs) {
      reportAt_ = now;
      ranges_[1].print(diag_.console, 2, now);
    }
    diag_.tick(now);  // I2C work only exists after a test command, which stops/disarms motors.
    diag_.console.flushSome();
  }

 private:
  void endButton(uint32_t now) {
    if (!buttonActive_) return;
    buttonDrive_.off();
    buttonActive_ = false;
    buttonOffAt_ = now;
    diag_.console.log("P1 motor drive off.");
  }

  void disarmButton(uint32_t now) {
    endButton(now);
    buttonArmed_ = released_ = false;
  }

  void armButton(uint32_t now) {
    if (buttonArmed_) { diag_.console.log("P1 button already armed."); return; }
    if (!buttonReady_ || !uart_.link.online(now) || lastButton_ != 1 ||
        elapsed(now, buttonRxAt_) >= config::kLinkTimeoutMs || diag_.busy()) {
      diag_.console.log("Requires jumper bench setup, live P1 link and released mapped button.");
      return;
    }
    buttonArmed_ = released_ = true;
    diag_.console.log("P1 button armed for motor 1.");
  }

  bool startButtonDrive(uint8_t duty) {
    constexpr size_t cycles = config::kButtonDriveMs * 1000UL / config::kButtonPeriodUs;
    rmt_item32_t pulses[cycles + 1]{};
    for (size_t i = 0; i < cycles; ++i) {
      auto& pulse = pulses[i];
      pulse.level0 = 1;
      // Full duty uses two HIGH segments. A zero duration would end the train early.
      pulse.level1 = duty == 100;
      pulse.duration0 = config::kButtonPeriodUs * (duty == 100 ? 50 : duty) / 100;
      pulse.duration1 = config::kButtonPeriodUs - pulse.duration0;
    }
    return buttonDrive_.begin(board::kDirection[0][0], board::kMotorRmt[0], pulses, cycles + 1) &&
           buttonDrive_.start();
  }

  void receiveButton(uint32_t now, uint8_t state) {
    buttonRxAt_ = now;
    const uint8_t previous = lastButton_;
    lastButton_ = state;
    if (!state) { disarmButton(now); return; }
    if (!buttonArmed_) return;
    if (state == 1) {
      endButton(now);
      released_ = true;
    } else if (state == 6 || (previous >= 2 && state != previous)) {
      endButton(now);
      released_ = false;
    } else if (released_) {
      released_ = false;
      if (elapsed(now, buttonOffAt_) < config::kMinMotorOffMs) return;
      const uint8_t duty = config::kButtonDuty[state - 2];
      if (!startButtonDrive(duty)) { disarmButton(now); return; }
      buttonStarted_ = now;
      buttonActive_ = true;
      activeDuty_ = duty;
      diag_.console.log("P1 button %u: motor 1 forward at %u%%.", state - 1, duty);
    }
  }

  bool motionPermitted(uint32_t now) const {
    return uart_.link.permitsMotion(now) && bool(Serial) && !diag_.busy();
  }

  void serviceTrial() {
    const uint32_t now = millis();
    trial_.tick(now, motionPermitted(now));
  }

  void peerChanged(uint32_t now) {
    const uint32_t session = uart_.link.peerSession();
    if (peer_ == session) return;
    if (peer_) trial_.stop(now, "Node rebooted or changed UART session");
    disarmButton(now);
    lastButton_ = 0;
    peer_ = session;
    ranges_[0] = {};
    trial_.invalidateSamples();
    remoteLedMask_ = 0;
  }

  void receive(const wire::Frame& frame, uint32_t now) {
    peerChanged(now);
    if (frame.kind == Kind::Stop || frame.kind == Kind::Test) {
      disarmButton(now);
      trial_.stop(now, frame.kind == Kind::Stop ? "Node requested STOP" : "Node restarted diagnostics");
      if (frame.kind == Kind::Test) diag_.start(now);
      else diag_.console.log("STOP received from Node. Motor enables commanded off.");
      uart_.link.acknowledge(frame);
      return;
    }
    if (!uart_.link.online(now)) return;
    if (frame.kind == Kind::ButtonArm) armButton(now);
    else if (frame.kind == Kind::P1Button) receiveButton(now, frame.data[0]);
    else if (frame.kind == Kind::LedState) {
      remoteLedMask_ = frame.data[0];
      ledsReceivedAt_ = now;
    } else if (frame.kind == Kind::SampleRequest && sonar_.start(now)) {
      shotSeq_ = frame.seq;
    } else if (frame.kind == Kind::Sample) {
      RangeSample sample;
      if (wire::get16(frame.data + 5) == 0 && wire::unpackSample(frame, sample)) {
        ranges_[0].sample(sample, now);
        trial_.sample(now, 0, sample);
      }
    }
  }

  void reportChanges(uint32_t now) {
    const bool online = uart_.link.online(now);
    if (online != wasOnline_) {
      wasOnline_ = online;
      diag_.linkStatus(uart_.link, now);
    }
    const auto state = trial_.state();
    if (state != lastState_) {
      if (state == State::Verify) diag_.console.log("Burst ended; both enables commanded off. Checking settled sonar readings.");
      if (state == State::Observe) diag_.console.log("Detected %d mm displacement. Check correct motor and direction, then motor result ok/fail.", int(trial_.movementMm()));
      if (state == State::Fault) diag_.console.log("Motor %u %s FAIL: %s. Off; fault latched.", trial_.motor() + 1,
                                                  trial_.reverse() ? "reverse" : "forward", trial_.fault());
      if (state == State::Idle && lastState_ == State::Confirm) diag_.console.log("No burst; the pending confirmation was cleared.");
      lastState_ = state;
    }
    if (online) {
      const uint8_t body[wire::kStatusSize] = {
          uint8_t(diag_.oled.bus()), uint8_t(diag_.oled.display()), uint8_t(diag_.leds.verdict()),
          uint8_t(trial_.verdict(0)), uint8_t(trial_.verdict(1)),
          uint8_t(buttonArmed_ ? 3 : state == State::Fault ? 2 : trial_.lockReason() ? 0 : 1), diag_.leds.appliedMask(), uint8_t(buttonActive_ ? activeDuty_ : 0)};
      if ((memcmp(body, lastReport_, sizeof(body)) != 0 || elapsed(now, sendAt_) >= config::kStatusPeriodMs) &&
          uart_.link.send(Kind::Status, body, sizeof(body))) {
        memcpy(lastReport_, body, sizeof(body));
        sendAt_ = now;
      }
    }
  }

  void motorStatus(uint32_t now) {
    diag_.console.log("P1 button bench: %s", buttonArmed_ ? "armed" : "disarmed");
    if (const char* reason = trial_.lockReason()) diag_.console.log("Motor tests LOCKED: %s.", reason);
    else {
      diag_.console.log("Motors %s | Node permit: %s | burst %u ms at %u%% enable duty", stateName(trial_.state()),
                        uart_.link.permitsMotion(now) ? "yes" : "no", config::kMotorBurstMs, config::kMotorDutyPercent);
      diag_.console.log("Recorded motor supply %u mV; external limit setting %u mA (no current sensor).",
                        board::kBench.measuredSupplyMv, board::kBench.supplyLimitSettingMa);
    }
    for (uint8_t i = 0; i < 2; ++i) {
      diag_.console.log("Motor %u: %s | forward: %s | reverse: %s", i + 1, label(trial_.verdict(i)),
                        label(trial_.direction(i, false)), label(trial_.direction(i, true)));
    }
    if (trial_.fault()) diag_.console.log("Latched: %s. Remove motor power and inspect before motor clear inspected.", trial_.fault());
  }

  void motorCommand(const Console::Command& cmd, uint32_t now) {
    if (cmd.count == 2 && is(cmd, 1, "status")) {
      motorStatus(now);
    } else if (cmd.count == 4 && is(cmd, 1, "prepare") && (is(cmd, 2, "1") || is(cmd, 2, "2")) &&
               (is(cmd, 3, "fwd") || is(cmd, 3, "rev"))) {
      const uint8_t motor = cmd.words[2][0] - '1';
      const bool reverse = is(cmd, 3, "rev");
      const uint32_t random = esp_random();
      const uint32_t token = random ? random : 1;
      if (const char* reason = trial_.prepare(now, motor, reverse, token, motionPermitted(now))) {
        diag_.console.log("Refused: %s.", reason);
      } else {
        const MotorSpec& spec = board::kBench.motors[motor];
        diag_.console.log("Motor %u %s: %s, rated %u mV; one %u ms burst at %u%% duty.", motor + 1,
                          reverse ? "reverse" : "forward", spec.model, spec.ratedMv, config::kMotorBurstMs, config::kMotorDutyPercent);
        diag_.console.log("Check wiring, supply/current limit, clearance and disconnect. To authorize: motor confirm %08lX (%u s).",
                          (unsigned long)token, config::kConfirmMs / 1000);
      }
    } else if (cmd.count == 3 && is(cmd, 1, "confirm")) {
      uint32_t token = 0;
      parseToken(cmd.words[2], token);
      if (const char* reason = trial_.confirm(now, token, motionPermitted(now))) diag_.console.log("Refused: %s.", reason);
      else diag_.console.log("Motor %u %s burst commanded. Press ! to abort.", trial_.motor() + 1, trial_.reverse() ? "reverse" : "forward");
    } else if (cmd.count == 3 && is(cmd, 1, "result") && confirmation(cmd, 2)) {
      const bool ok = is(cmd, 2, "ok");
      if (!trial_.recordObservation(now, ok)) diag_.console.log("No completed displacement test awaiting an observation.");
      else diag_.console.log("Motor %u %s %s (operator).", trial_.motor() + 1, trial_.reverse() ? "reverse" : "forward", ok ? "PASS" : "FAIL");
    } else if (cmd.count == 3 && is(cmd, 1, "clear") && is(cmd, 2, "inspected")) {
      if (!trial_.clearFault()) diag_.console.log("No latched fault to clear.");
      else {
        trial_.invalidateSamples();
        diag_.console.log("Fault cleared; disarmed. Results retained. A new prepare and confirmation are required.");
      }
    } else {
      diag_.console.log("Motor commands: motor status | motor prepare <1|2> <fwd|rev> | motor confirm <token>");
      diag_.console.log("motor result ok/fail | motor clear inspected | ! or stop");
    }
  }

  void command(const Console::Command& cmd, uint32_t now) {
    if (cmd.count == 2 && is(cmd, 0, "button") && is(cmd, 1, "arm")) {
      armButton(now);
    } else if (cmd.count == 1 && is(cmd, 0, "test")) {
      disarmButton(now);
      trial_.stop(now, "operator restarted diagnostics");
      diag_.start(now);
      diag_.console.log("P2 test started. Node schedules both sonar sensors and monitors all buttons.");
    } else if (cmd.count == 1 && (is(cmd, 0, "stop") || is(cmd, 0, "!"))) {
      disarmButton(now);
      trial_.stop(now, "operator stopped trial");
      diag_.console.log("STOP: both enables commanded off; any pending confirmation cleared.");
    } else if (cmd.count == 1 && is(cmd, 0, "status")) {
      diag_.status();
      diag_.linkStatus(uart_.link, now);
      ranges_[0].print(diag_.console, 1, now);
      ranges_[1].print(diag_.console, 2, now);
      motorStatus(now);
    } else if (is(cmd, 0, "motor")) motorCommand(cmd, now);
    else if (!diag_.command(cmd)) {
      diag_.console.log("Commands: help | test | status | stream on/off | oled ok/fail | led <1-4> ok/fail");
      diag_.console.log("Enter motor for bench commands or ! to stop. Test mode only.");
    }
  }

  PulseOutput buttonDrive_;
  bool buttonReady_ = false, buttonArmed_ = false, buttonActive_ = false, released_ = false;
  uint32_t buttonStarted_ = 0, buttonOffAt_ = 0, buttonRxAt_ = 0;
  uint8_t lastButton_ = 0, activeDuty_ = 0;
  MotorDriver driver_;
  MotorTrial trial_{board::kBench, driver_.outputs()};
  Uart uart_{wire::Role::Nano};
  Sonar sonar_;
  Diagnostics diag_;
  RangeLog ranges_[2];
  uint32_t peer_ = 0, sendAt_ = 0, reportAt_ = 0;
  uint32_t ledsReceivedAt_ = 0;
  uint16_t shotSeq_ = 0;
  uint8_t remoteLedMask_ = 0;
  uint8_t lastReport_[wire::kStatusSize] = {0xff};
  State lastState_ = State::Idle;
  bool wasOnline_ = false;
};

Controller controller;
}  // namespace

void setup() { controller.begin(); }
void loop() { controller.tick(); }
