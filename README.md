# ESP32 horse racing: Test Mode

`nodemcu/` and `nano/` are separate PlatformIO projects. Both use `shared/` for the Arduino C++ diagnostics, UART protocol, and motor interlocks. This milestone stops at hardware bring-up; there are no calibration or race commands. The default build refuses every powered motor trial.

## Wire the controllers

| Signal | NodeMCU-32S | Arduino Nano ESP32 |
| --- | --- | --- |
| Player 1 buttons 1–4 | GPIO35, 34, 36, 39 | — |
| Player 2 buttons 1–4 | GPIO16, 17, 18, 19 | — |
| Player 1 LEDs 1–4 | GPIO25, 26, 27, 32 | — |
| Player 2 LEDs 1–4 | — | D2, D3, D4, D5 |
| Local SSD1306 SDA / SCL | GPIO21 / GPIO22 | A4 / A5 |
| Local HC-SR04 TRIG / ECHO | GPIO23 / GPIO33 | A2 / A3 |
| Interboard UART TX / RX | GPIO13 / GPIO14 | D8 / D9 |
| L298N motor 1 IN1 / IN2 | — | D6 / D7 |
| L298N motor 2 IN3 / IN4 | — | D10 / D11 |
| Optional ENA / ENB for powered testing | — | D12 / A0, **jumpers removed** |

- Cross the UART lines: GPIO13 TX → D9 RX; D8 TX → GPIO14 RX. Tie both board grounds together. This is 3.3 V UART (`115200`, 8N1), separate from each board’s USB console.
- Wire each normally open button to GND when pressed. P2 uses internal pull-ups. Give each P1 input its own external pull-up to **3.3 V** (e.g. 10 kΩ); GPIO34–39 cannot provide internal pull-ups. Use a series resistor with each active-high LED.
- The P1 OLED uses `0x3C` and the P2 OLED uses `0x3D`. Set each address with `kOledAddress` in `Board.h`. Keep the I²C pull-ups at 3.3 V. Set the board’s `kOledHeight` to `32` for a 128×32 SSD1306. The default is 64.
- Power each HC-SR04 from regulated 5 V as required by its module. Level shift **both ECHO signals** down to 3.3 V logic before connecting them to the ESP32s. Never connect a 5 V ECHO directly. Tie sensor ground to board ground. Verify that the module accepts a 3.3 V TRIG or use an appropriate level translator.
- Leave motor power disconnected through the initial checks, reset, and upload. When eventually using the external motor supply, share ground with the Nano and driver. Check the module’s logic supply, flyback protection and regulator jumper independently of ENA/ENB; do not feed the driver’s 5 V rail into an ESP32 I/O pin.

Edit pins, display geometry and the bench setup in [`shared/src/race/Board.h`](shared/src/race/Board.h). Timings, sonar limits and burst settings live in [`shared/src/race/Config.h`](shared/src/race/Config.h). The Nano config uses [Arduino header labels](https://docs.arduino.cc/resources/pinouts/ABX00083-full-pinout.pdf); the pinned core handles remapping for its Arduino APIs.

## Build, flash, and open the consoles

Install PlatformIO Core or the VS Code extension. From this repository root:

```sh
pio run -d nodemcu
pio run -d nano
pio device list
```

1. Identify each port by plugging in one board at a time. Close any serial monitors before uploading. Flash the NodeMCU, replacing `NODE_PORT` with the port from `pio device list`:

   ```sh
   pio run -d nodemcu -t upload --upload-port NODE_PORT
   ```

   If auto reset fails, use the NodeMCU BOOT/EN buttons to enter serial download mode and retry.

2. For the Nano, **double press RESET** to enter its Arduino bootloader. With one Nano ESP32 in DFU mode connected, run:

   ```sh
   pio run -d nano -t upload
   ```

   This uses [PlatformIO’s Nano ESP32 DFU uploader](https://docs.platformio.org/en/stable/boards/espressif32/arduino_nano_esp32.html#uploading). If DFU is not found, check the USB data cable and [Arduino’s bootloader instructions](https://support.arduino.cc/hc/en-us/articles/22545229351964-Upload-a-sketch-to-a-board-running-MicroPython). Reset once if the Nano stays in the bootloader afterward and check for its new USB serial port.

3. In two terminals, substitute the actual port names. Both monitors use `115200`; press Enter to submit commands. `!` stops as soon as the monitor sends that byte. Keep DTR and RTS enabled on the Nano monitor (set in its project).

   ```sh
   pio device monitor -d nodemcu -p NODE_PORT
   pio device monitor -d nano -p NANO_PORT
   ```

## Run the unpowered diagnostics

1. Send `status` to each board. Both should report `UART heartbeat: PASS` within a second of connecting. A board must receive the peer’s ping and an ACK for its own ping. With motor power disconnected, unplug either UART signal temporarily and check for `FAIL` on both monitors. Restore it and check that both return to `PASS`.
2. Send `test` on NodeMCU. It starts the P1 test and requests the P2 test over UART. `Nano TEST PASS: acknowledged` means the Nano accepted the request. If the boards aren’t linked yet, `test` on the Nano starts its local OLED and LED routines independently.
3. Each OLED should show `P1 TEST` or `P2 TEST`, a border and bottom bars. On **its own console**, enter `oled ok` if the pattern is correct, or `oled fail` if it is blank or wrong after a successful I²C check. If the I²C check fails, examine power, address, pull-ups and SDA/SCL before retesting.
4. Watch LEDs 1–4 light in order with the others off. On the corresponding board, enter `led 1 ok`, `led 2 ok`, `led 3 ok`, `led 4 ok` for the steps you saw, or `led N fail` for a wrong step. The test waits for these observations before reporting a passing bank.
5. When the LED sequence finishes, send `pair` on the NodeMCU. For each lit LED, press and release the button you want to assign to it. Pairing goes through all four P1 LEDs and then all four P2 LEDs. The next LED lights as soon as the current choice is accepted. Use each button once within its player group. `pair status` repeats the current step and shows the GPIO assignments once saved. `pair cancel` discards the current choices and restores the previous mapping.
6. Put a flat target within range of each sonar. Move one target at a time and verify the corresponding distance changes. NodeMCU streams both sensors once a second after `test`; Nano streams its local sensor. `stream on` / `stream off` control console output, and `status` prints the latest values and counts at any time. The Node schedules alternating pings to reduce crosstalk. Nano sonar requires the live Node link.

| Report | Meaning |
| --- | --- |
| `OLED I2C PASS` | The device acknowledged. `screen: MANUAL CHECK` stays until someone sees the pattern. |
| `LEDs: PASS` | All four steps were confirmed on that board. `MANUAL CHECK` means one or more have no observation. |
| `buttons: PASS` | All four inputs in that group showed a debounced press followed by release. Check the console mapping against each physical switch. |
| `sonar PASS: VALID` | The latest full echo gave a distance in 20–4000 mm. Compare against a known target as a hardware check. |
| `sonar FAIL: OUT OF RANGE` | An echo ended but converted to a distance outside the configured sensor range. |
| `sonar FAIL: TIMEOUT` | No complete echo within 30 ms, or ECHO was already high at trigger time. Check target, alignment, wiring and level shift. |
| `sonar FAIL: STALE` / `TRIGGER ERROR` | A prior result stopped updating, or trigger hardware failed to initialize/start. `NOT SAMPLED` means no result yet. |
| `UART heartbeat FAIL` | Missing a recent ping or matching ACK from the other board. Check both TX/RX paths and shared ground. `self-echoes` counts the board’s own packets coming back on RX; a loop between its TX and RX cannot pass. |
| `Motor: MANUAL CHECK` | Powered testing is locked or one of the two directions lacks completed sensor and human evidence. |

`range`, `timeout`, and `error` are cumulative sample counts since boot (since the peer session changed for remote readings). A valid current sample does not erase earlier errors. `test` restarts the LED/OLED checks and resets NodeMCU button evidence. Record `status` from both monitors if you need to keep results across resets.

The NodeMCU saves the mapping after the eighth accepted choice. After pairing, hold a button to light its assigned LED and release it to turn the LED off. Check the physical LEDs in the order you chose. `status` shows the observed press and release positions for each player, with `-` for a button that still needs a check.

## Prepare powered motor tests

**With ENA/ENB jumpered HIGH, this firmware will not test powered motors.** The supplied IN1–IN4 wiring selects direction; the firmware does not implement input PWM. Input PWM would require a deliberate switching strategy and a safe disable path. This build applies a finite pulse train to each **enable** so it can remove drive independently of the direction inputs. Per the [L298 datasheet](https://www.st.com/resource/en/datasheet/l298.pdf), enable LOW allows coasting; direction inputs equal with enable HIGH invoke fast braking.

Before changing the factory-locked `kBench` values:

1. With power disconnected, identify **both exact motor models and voltage ratings**. Verify the L298N module’s operating voltage and current ratings, wiring and flyback components. Remove the ENA/ENB HIGH jumpers; confirm neither enable pin is still tied to 5 V. Wire `D12 → ENA` and `A0 → ENB` with an external pull-down on **each** enable (e.g. 10 kΩ to GND) so reset and unplugged GPIOs leave the bridges disabled. Verify reset and idle levels before applying motor power.
2. Measure the external motor supply voltage at the driver input. Enter it in millivolts and keep it within both motor ratings and the module’s operating range. Use an external current limit sized from the actual motor, driver, wiring, and supply limits. Startup and stall current can exceed running current; verify the limiter handles transients, including energy from any output capacitance. If you cannot establish a suitable limit, leave the motors disconnected. The L298N and firmware PWM do not provide current limiting. `supplyLimitSettingMa` records the external limit **setting**; there is no current sensor.
3. Decouple the race load. Secure each motor and use a freely moving, observable bench target for its sensor (P1 sensor for motor 1, P2 for motor 2). Keep the target and mechanism away from travel stops with enough clearance in **both** directions. Sonar samples intermittently and the motor can coast, so the sonar window cannot serve as a physical limit switch. With power off, check by hand that the target tracks motion and determine a safe observation window in millimeters for each motor. A bare spinning shaft without a changing sonar target cannot pass the sensor check.
4. Fill in `kBench` in `Board.h`: enable wiring flag, measured supply voltage, external limit setting, exact motor models, rated voltages and both safe sensor windows. Rebuild and flash Nano with motor power disconnected. Verify `motor status` shows `disarmed`. With the motor supply disconnected, use a scope or logic analyzer to verify both enables are low during reset and idle, and that a confirmed trial produces a finite train only on its selected enable. Expect no displacement and a latched fault on a dry trial; inspect and clear it before applying power.
5. Provide an accessible physical motor supply disconnect. Verify the mechanism is clear and the external current limit is set before powering the driver. Start with the configured short, low-duty settings. If anything binds, heats, moves unexpectedly or hits a stop, cut motor power and inspect. Do not increase duty to push through a stall.

## Run one burst at a time

After the unpowered checks and setup above:

1. On **NodeMCU**, send `link resume` to allow the Nano to ask for trial confirmation. This does not move anything. Boot, UART loss, re-running the Node test, and Node `stop` clear permission.
2. On **Nano**, send one of `motor prepare 1 fwd`, `motor prepare 1 rev`, `motor prepare 2 fwd`, or `motor prepare 2 rev`. The target must be stationary and within its safe window, with two recent valid readings and an active two-way heartbeat. Keep the Nano USB monitor open.
3. Check the rig and confirm with `motor confirm TOKEN` using the exact one-time token printed by the Nano (20 seconds). The default is one **120 ms, 20% duty** burst at 250 Hz. `Config.h` allows shorter bursts in 4 ms steps and lower duty. Compile-time caps are **160 ms and 25%**, and both bridges must stay off for at least **1.5 s** between bursts. The whole train is stored in the ESP32-S3 RMT peripheral with an end marker and idle LOW; it finishes even if the main loop stops servicing it. A hardware supply disconnect remains the primary emergency stop.
4. After power is removed, the firmware checks for two settled in-window readings showing sustained displacement (default at least 12 mm in the same sense). It does not infer correct physical direction. Only if the right motor moved in the expected direction, send `motor result ok` within 60 seconds; otherwise send `motor result fail`. Repeat with a new token for each direction of each motor. Both directions must pass for a motor to report `PASS`.

Use **`!`** or `stop` on Nano to cancel immediately; Node `!` or `stop` requests an acknowledged remote stop and inhibits subsequent authorizations. A stopped motor may coast. Any detected loss of the UART or USB interlock, invalid/stale selected sensor, missed movement deadline, or aborted active trial shuts both enables off and latches a fault. After removing motor power and inspecting the cause, `motor clear inspected` releases the latch; it never restarts a test and it retains the failed verdict until retested. There is no automatic retry or automatic reversal.

## Software verification

```sh
sh tests/run.sh
```

The host suite covers framed CRC packets, malformed/truncated frame handling, two-way heartbeats, retries and duplicate suppression, button debounce and rollover, sonar result decoding, and motor interlocks with simulated outputs. It does not verify the electrical behavior. Physical acceptance requires the observed diagnostics above before any calibration or gameplay.

The UART wire format is `00 COBS(version | type | sender role | boot session | sequence | length | payload | CRC-16/CCITT-FALSE) 00`, with little-endian fields. Flash both boards from this checkout so they use the same protocol version. Both ignore their own packets and packets from the wrong role. Pings, control commands and acknowledgments carry sequence/session IDs; commands retry at most three times and are applied once per session/sequence. Sensor and status streams are best-effort and marked stale on timeout. A Nano shot is reported with its Node request sequence. Neither board uses blocking echo reads or `delay()`; milliseconds drive the state machines, with peripheral timing for trigger/enable pulses and an interrupt for echo edges.

### P1 button bench mode with ENA/ENB jumpers

With both enables jumpered HIGH and the motor secured for a sensorless bench test, enter `button arm` on either board while the button paired to P1 LED 1 is released. The four paired P1 buttons drive motor 1 forward through D6 at 25%, 50%, 75%, and 100% input PWM in LED order. The P1 OLED lists each button, its GPIO (G), and duty. RUNNING means the Nano reported active drive. Multiple buttons stop drive until all are released. Connect a dedicated Nano GND to driver GND with power disconnected. Release stops the drive, with a one-second hardware cap and 1.5 seconds off before another press. Holding the button cannot restart it. The Nano boots disarmed. STOP, diagnostics, pairing, or loss of button updates disarm this mode. The standard sensor-validated motor trials have separate enable-pin wiring requirements.
