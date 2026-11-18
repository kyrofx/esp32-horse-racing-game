#include <Preferences.h>

#include <race/ButtonPairing.h>
#include <race/ButtonTracker.h>
#include <race/Diagnostics.h>
#include <race/Hardware.h>

namespace {
using namespace race;
using wire::Kind;
constexpr char kMappingNamespace[] = "race-pairing";
constexpr char kMappingKey[] = "map-v2";

class Controller {
 public:
  void begin() {
    diag_.begin();
    diag_.oled.start();
    uart_.begin();
    sonar_.begin();
    for (uint8_t i = 0; i < 8; ++i) {
      pinMode(board::kButtons[i], board::buttonMode(board::kButtons[i]));
    }
    resetButtons(millis());
    loadMapping();
    reportPairing();
  }

  void tick() {
    uart_.poll([this](const wire::Frame& frame) { receive(frame, millis()); });
    peerChanged();
    Console::Command cmd{};
    if (diag_.console.read(cmd)) command(cmd, millis());

    const uint32_t now = millis();
    uart_.link.tick(now, permit_);
    reportLink(now);
    updateButtonLeds(now);
    RangeSample sample;
    if (sonar_.poll(sample)) {
      ranges_[0].sample(sample, now);
      uint8_t data[7];
      wire::packSample(sample, data);
      uart_.link.send(Kind::Sample, data, sizeof(data));
    }
    scheduleSonar(now);
    if (diag_.stream && elapsed(now, reportAt_) >= config::kConsoleReportMs) {
      reportAt_ = now;
      ranges_[0].print(diag_.console, 1, now);
      ranges_[1].print(diag_.console, 2, now);
    }
    diag_.tick(now);
    if (motorScreen_ && !diag_.oled.busy()) {
      const bool fresh = uart_.link.online(now) && freshNanoStatus(now);
      const uint8_t duty = fresh ? remote_[7] : 0;
      char rows[4][22] = {"P1 MOTOR 1"};
      if (duty) snprintf(rows[1], sizeof(rows[1]), "RUNNING %u%% PWM", duty);
      else snprintf(rows[1], sizeof(rows[1]), "%s", !fresh ? "NO LINK" : remote_[5] == 3 ? "READY" : "DISARMED");
      for (uint8_t row = 0; row < 2; ++row) {
        char* line = rows[row + 2];
        for (uint8_t col = 0; col < 2; ++col) {
          const uint8_t i = 2 * row + col;
          const uint8_t input = mapping_.inputForLed[i];
          if (input >= 4) snprintf(line + col * 11, 22 - col * 11, "%u UNPAIRED", i + 1);
          else snprintf(line + col * 11, 22 - col * 11, "%u G%02u %u%%", i + 1,
                   board::kButtons[input], config::kButtonDuty[i]);
          if (!col) memset(line + strlen(line), ' ', 11 - strlen(line));
        }
      }
      diag_.oled.motorScreen(rows, diag_.console);
    }
    diag_.console.flushSome();
  }

 private:
  static bool pressed(uint8_t i) { return digitalRead(board::kButtons[i]) == board::kButtonPressed; }

  void resetButtons(uint32_t now) {
    for (uint8_t i = 0; i < 8; ++i) buttons_[i].reset(pressed(i), now);
  }

  void loadMapping() {
    Preferences storage;
    if (!storage.begin(kMappingNamespace, true)) return;
    ButtonMapping loaded;
    if (storage.getBytesLength(kMappingKey) == sizeof(loaded.inputForLed) &&
        storage.getBytes(kMappingKey, loaded.inputForLed, sizeof(loaded.inputForLed)) == sizeof(loaded.inputForLed) &&
        loaded.valid()) {
      mapping_ = loaded;
    }
  }

  bool saveMapping() {
    Preferences storage;
    return storage.begin(kMappingNamespace, false) &&
           storage.putBytes(kMappingKey, mapping_.inputForLed, sizeof(mapping_.inputForLed)) == sizeof(mapping_.inputForLed);
  }

  void reportPairing() {
    if (pairing_.active()) {
      const uint8_t step = pairing_.step();
      diag_.console.log("P%u LED %u selected. Press and release the P%u button you want while that LED is lit.",
                        step / 4 + 1, step % 4 + 1, step / 4 + 1);
      return;
    }
    if (!mapping_.valid()) {
      diag_.console.log("Buttons awaiting assignment. Enter pair to select a button for each LED.");
      return;
    }
    for (uint8_t bank = 0; bank < 2; ++bank) {
      const auto* assigned = mapping_.inputForLed + bank * 4;
      const auto* pins = board::kButtons + bank * 4;
      diag_.console.log("P%u LEDs 1 through 4 use GPIOs %u, %u, %u, %u in that order.", bank + 1,
                        pins[assigned[0]], pins[assigned[1]], pins[assigned[2]], pins[assigned[3]]);
    }
  }

  bool freshNanoStatus(uint32_t now) const {
    return remoteStatus_ && uart_.link.online(now) && elapsed(now, remoteAt_) < config::kStatusFreshMs;
  }

  void updatePairing(uint32_t now, uint8_t pressed) {
    if (!pairing_.active()) return;
    const uint8_t step = pairing_.step();
    const uint8_t indicator = 1u << (step % 4);
    const bool ready = step < 4 ? diag_.leds.appliedMask() == indicator
                                : freshNanoStatus(now) && remote_[6] == indicator;
    switch (pairing_.update(pressed, ready)) {
      case ButtonPairing::Event::None:
        return;
      case ButtonPairing::Event::Multiple:
        diag_.console.log("Release all buttons and choose one for this LED.");
        return;
      case ButtonPairing::Event::WrongPlayer:
        diag_.console.log("Use one of the P%u buttons for this LED.", step / 4 + 1);
        return;
      case ButtonPairing::Event::Reused:
        diag_.console.log("That button already has an LED. Choose an unused button.");
        return;
      case ButtonPairing::Event::Assigned:
        break;
    }
    const uint8_t input = pairing_.draft().inputForLed[step];
    diag_.console.log("Paired P%u LED %u with button GPIO%u.", step / 4 + 1, step % 4 + 1,
                      board::kButtons[(step / 4) * 4 + input]);
    if (!pairing_.active()) {
      mapping_ = pairing_.draft();
      diag_.console.log(saveMapping() ? "All eight assignments saved." : "Assignments active for this boot. Flash storage failed.");
    }
    reportPairing();
  }

  void updateButtonLeds(uint32_t now) {
    uint8_t pressedMasks[2]{};
    for (uint8_t i = 0; i < 8; ++i) {
      auto& button = buttons_[i];
      if (button.update(pressed(i), now)) {
        diag_.console.log("P%u button %u (GPIO%u) %s%s", i / 4 + 1, i % 4 + 1, board::kButtons[i],
                          button.pressed() ? "PRESSED" : "RELEASED", button.passed() ? " | PASS" : "");
      }
      if (button.pressed()) pressedMasks[i / 4] |= 1u << (i % 4);
    }
    updatePairing(now, pressedMasks[0] | (pressedMasks[1] << 4));
    uint8_t localMask = 0, remoteMask = 0;
    if (pairing_.active()) {
      const uint8_t step = pairing_.step();
      const uint8_t indicator = 1u << (step % 4);
      if (step < 4) localMask = indicator;
      else remoteMask = indicator;
    } else {
      localMask = mapping_.ledMask(0, pressedMasks[0]);
      remoteMask = mapping_.ledMask(1, pressedMasks[1]);
    }
    diag_.leds.setLedMask(localMask, diag_.console);
    sendNanoLeds(now, remoteMask);
    const uint8_t state = pairing_.active() || diag_.busy() || !mapping_.valid()
                              ? 0 : config::buttonDriveState(localMask);
    if (uart_.link.online(now) &&
        (state != lastButtonState_ || elapsed(now, buttonSentAt_) >= config::kLedUpdateMs) &&
        uart_.link.send(Kind::P1Button, &state, 1)) {
      lastButtonState_ = state;
      buttonSentAt_ = now;
    }
  }

  void sendNanoLeds(uint32_t now, uint8_t mask) {
    if (!uart_.link.online(now)) {
      sentLedState_ = false;
      return;
    }
    if (sentLedState_ && mask == lastLedMask_ && elapsed(now, ledsSentAt_) < config::kLedUpdateMs) return;
    if (uart_.link.send(Kind::LedState, &mask, 1)) {
      sentLedState_ = true;
      lastLedMask_ = mask;
      ledsSentAt_ = now;
    }
  }

  void peerChanged() {
    const uint32_t session = uart_.link.peerSession();
    if (peer_ == session) return;
    if (peer_) {
      permit_ = false;
      diag_.console.log("Nano reboot/session change: motion permission cleared.");
    }
    peer_ = session;
    remoteStatus_ = false;
    ranges_[1] = {};
    waiting_ = false;
    sentLedState_ = false;
  }

  void receive(const wire::Frame& frame, uint32_t now) {
    peerChanged();
    if (!uart_.link.online(now)) return;
    RangeSample sample;
    if (frame.kind == Kind::Sample && waiting_ && wire::get16(frame.data + 5) == shotSeq_ &&
        elapsed(now, slotAt_) < config::kRemoteShotTimeoutMs && wire::unpackSample(frame, sample)) {
      ranges_[1].sample(sample, now);
      waiting_ = false;
    } else if (frame.kind == Kind::Status) {
      for (uint8_t i = 0; i < 5; ++i) if (frame.data[i] > uint8_t(Verdict::Fail)) return;
      if (frame.data[5] > 3 || (frame.data[6] > config::kLedMask && frame.data[6] != config::kLedSweepActive)) return;
      if (frame.data[7] > 100 || frame.data[7] % 25 || (frame.data[7] && frame.data[5] != 3)) return;
      if (frame.data[5] == 3 && !diag_.busy()) motorScreen_ = true;
      memcpy(remote_, frame.data, sizeof(remote_));
      remoteStatus_ = true;
      remoteAt_ = now;
    }
  }

  void scheduleSonar(uint32_t now) {
    if (elapsed(now, slotAt_) < config::kSonarSlotMs) return;
    // A late Nano response gets its own quiet window; an absent reply cannot block P1 indefinitely.
    if (waiting_ && elapsed(now, slotAt_) < config::kRemoteShotTimeoutMs) return;
    if (ranges_[1].last.state != RangeState::None && elapsed(now, ranges_[1].receivedAt) < config::kSonarQuietMs) return;
    waiting_ = false;
    slotAt_ = now;
    if (nanoSlot_ && uart_.link.online(now)) waiting_ = uart_.link.send(Kind::SampleRequest, nullptr, 0, &shotSeq_);
    else if (!nanoSlot_) sonar_.start(now);
    nanoSlot_ = !nanoSlot_;
  }

  void reportLink(uint32_t now) {
    const bool online = uart_.link.online(now);
    if (online != wasOnline_) {
      if (!online) {
        permit_ = false;
        diag_.console.log("Motion permission cleared after UART loss.");
      }
      wasOnline_ = online;
      diag_.linkStatus(uart_.link, now);
    }
    wire::Link::Receipt receipt{};
    if (uart_.link.takeReceipt(receipt)) {
      diag_.console.log("Nano %s %s: %s", receipt.kind == Kind::Stop ? "STOP" : "TEST", receipt.acked ? "PASS" : "FAIL",
                        receipt.acked ? "acknowledged" : "ACK timeout/session change; check Nano monitor");
    }
  }

  void status(uint32_t now) {
    diag_.status();
    reportPairing();
    for (uint8_t group = 0; group < 2; ++group) {
      char observed[5] = "----";
      bool all = true;
      for (uint8_t i = 0; i < 4; ++i) {
        if (buttons_[group * 4 + i].passed()) observed[i] = '1' + i;
        else all = false;
      }
      diag_.console.log("P%u buttons: %s | observed press/release [%s]", group + 1, label(all ? Verdict::Pass : Verdict::Manual), observed);
    }
    diag_.linkStatus(uart_.link, now);
    diag_.console.log("Node motion permission: %s", permit_ ? "allowed (Nano confirmation still required)" : "inhibited");
    ranges_[0].print(diag_.console, 1, now);
    ranges_[1].print(diag_.console, 2, now);
    if (freshNanoStatus(now)) {
      diag_.console.log("P2 OLED I2C: %s | screen: %s | LEDs: %s", label(Verdict(remote_[0])), label(Verdict(remote_[1])), label(Verdict(remote_[2])));
      diag_.console.log("Nano motor 1: %s | motor 2: %s | %s", label(Verdict(remote_[3])), label(Verdict(remote_[4])),
                        remote_[5] == 3 ? "P1 button armed" : remote_[5] == 2 ? "FAULT LATCHED" : remote_[5] == 1 ? "configured" : "locked");
    } else {
      diag_.console.log("P2 OLED, LEDs, motors: MANUAL CHECK (no current Nano report).");
    }
  }

  void cancelPairing() {
    if (!pairing_.active()) return;
    pairing_.cancel();
    diag_.console.log("Pairing canceled. Any saved assignments are active.");
  }

  void pairCommand(const Console::Command& cmd, uint32_t now) {
    if (cmd.count == 1) {
      if (pairing_.active()) {
        reportPairing();
        return;
      }
      if (diag_.leds.busy()) {
        diag_.console.log("Wait for the P1 LED test sequence to finish, then enter pair.");
        return;
      }
      if (permit_) uart_.link.request(Kind::Stop, now);
      permit_ = false;
      pairing_.start();
      reportPairing();
    } else if (cmd.count == 2 && is(cmd, 1, "cancel")) {
      cancelPairing();
    } else if (cmd.count == 2 && is(cmd, 1, "status")) {
      reportPairing();
    } else {
      diag_.console.log("Pairing commands are pair, pair status and pair cancel.");
    }
  }

  void command(const Console::Command& cmd, uint32_t now) {
    if (cmd.count == 2 && is(cmd, 0, "button") && is(cmd, 1, "arm")) {
      motorScreen_ = true;
      if (uart_.link.online(now) && uart_.link.send(Kind::ButtonArm, nullptr, 0))
        diag_.console.log("Requested P1 button arm. Check Nano state with status.");
    } else if (is(cmd, 0, "pair")) {
      pairCommand(cmd, now);
    } else if (cmd.count == 1 && is(cmd, 0, "test")) {
      cancelPairing();
      permit_ = false;
      motorScreen_ = false;
      resetButtons(now);
      diag_.start(now);
      if (!uart_.link.online(now) || !uart_.link.request(Kind::Test, now)) {
        diag_.console.log("P1 test started. Connect UART and re-run test, or run test locally on Nano.");
      } else {
        remoteStatus_ = false;
        diag_.console.log("Testing both boards; confirm P2 LEDs and OLED on the Nano console.");
      }
      diag_.console.log("Press and release all eight buttons; keep motor power off for these checks.");
    } else if (cmd.count == 1 && (is(cmd, 0, "stop") || is(cmd, 0, "!"))) {
      cancelPairing();
      permit_ = false;
      uart_.link.request(Kind::Stop, now);
      diag_.console.log("STOP: Nano stop requested; motion permission inhibited.");
    } else if (cmd.count == 2 && is(cmd, 0, "link") && is(cmd, 1, "resume")) {
      if (pairing_.active()) {
        diag_.console.log("Finish or cancel button pairing before allowing motor trials.");
        return;
      }
      permit_ = uart_.link.online(now);
      diag_.console.log(permit_ ? "Motion permission sent on next heartbeat; arm trials locally on Nano." : "Need a live two-way UART heartbeat.");
    } else if (cmd.count == 1 && is(cmd, 0, "status")) status(now);
    else if (!diag_.command(cmd)) {
      diag_.console.log("Commands: help | test | status | stream on/off | oled ok/fail | led <1-4> ok/fail");
      diag_.console.log("Button assignments: pair | pair status | pair cancel");
      diag_.console.log("Motor interlock: ! or stop | link resume (use Nano console to prepare/confirm a burst)");
    }
  }

  uint8_t lastButtonState_ = 0xff;
  uint32_t buttonSentAt_ = 0;
  bool motorScreen_ = true;
  Diagnostics diag_;
  Uart uart_{wire::Role::Node};
  Sonar sonar_;
  ButtonTracker buttons_[8];
  ButtonPairing pairing_;
  ButtonMapping mapping_;
  RangeLog ranges_[2];
  uint32_t slotAt_ = 0, reportAt_ = 0, remoteAt_ = 0, peer_ = 0;
  uint32_t ledsSentAt_ = 0;
  uint16_t shotSeq_ = 0;
  uint8_t remote_[wire::kStatusSize]{};
  uint8_t lastLedMask_ = 0;
  bool sentLedState_ = false;
  bool nanoSlot_ = false, waiting_ = false, remoteStatus_ = false, permit_ = false, wasOnline_ = false;
};

Controller controller;
}  // namespace

void setup() { controller.begin(); }
void loop() { controller.tick(); }
