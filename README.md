# Spool2Spool Controller

Dual-motor arm/tension controller: an ESP-WROOM-32 devkit driving two 24V geared DC motors
through BTS7960/IBT-2 drivers, with an I2C LCD for local readout/control and a separate
Wemos D1 mini node reading the arm angle right at the pivot.

## What it does

- **Motor 2** has the only encoder (hall sensor/pulse feedback) in the system: closed loop
  constant speed control, targeting a speed in m/min (converted to RPM using the spool
  diameter). Pot 2 sets this target continuously in auto mode, and drives the PWM directly
  (open loop) in manual mode — same knob, different meaning depending on mode. A
  proportional controller corrects the PWM duty once per second.
- **Motor 1** has no encoder of its own. Its baseline PWM continuously tracks motor 2's PWM,
  keeping both motors nominally speed-matched, then gets a small proportional trim based on
  the arm's angle to hold that angle at a live-adjustable target. The angle itself is measured
  by an AS5600 magnetic sensor mounted right at the arm pivot, read by a separate Wemos D1
  mini, and pushed to the ESP32 over a one-way serial link — see
  [Wemos D1 mini angle-sensor node](#wemos-d1-mini-angle-sensor-node) below. The angle trim is
  deliberately small (capped, see `angleTrimLimitPWM`) so a noisy reading right at startup
  can only nudge motor 1 a little, never throw it far from motor 2's PWM. If motor 2's PWM
  is 0 (e.g. a 0 m/min setpoint), motor 1 forces to 0 as well regardless of angle error —
  the trim never applies on top of a stopped motor 2.
- Both motors support **manual override**, driving speed directly from a potentiometer instead
  of the automatic logic.
- Both motor outputs stay at **zero on power-up** until SELECT is pressed once.
- Local UI: LEFT/RIGHT toggle manual/auto mode per motor. SELECT starts the system, then
  (once started) cycles which value UP/DOWN adjusts: spool diameter, then motor 1's target
  angle — a transient screen shows the selected value and its new setting each time it
  changes. Motor 2's target speed isn't in this cycle; pot 2 sets that continuously instead
  (see above). The 20x4 I2C LCD otherwise shows an "EMI Twente" splash
  screen at boot, a "press SELECT to start" prompt, then live speed/mode2 on the top two
  lines and angle/mode1 on the bottom two.

## Hardware

- ESP-WROOM-32 devkit (30-pin), 3.3V logic throughout
- 20x4 I2C LCD (PCF8574-based backpack, address 0x27 or 0x3F depending on the module)
- 5 discrete pushbuttons (LEFT/RIGHT/UP/DOWN/SELECT), each to GND, using the ESP32's internal
  pull-ups — no external resistors, no resistor-ladder shield
- 2x 24V geared DC motors, each driven through a BTS7960/IBT-2 module (PWM speed control only,
  R_EN/L_EN tied high in hardware, no direction control needed)
- Hall sensor/encoder on motor 2's shaft, 1 pulse per revolution, interrupt driven — the
  only encoder in the system; motor 1 has none
- 2x potentiometers: pot 1 is motor 1's manual PWM override; pot 2 is motor 2's manual PWM
  override in manual mode, and its target speed setpoint in auto mode
- A separate **Wemos D1 mini + AS5600** node mounted at the arm pivot, sending the angle over
  a serial link (see below) — no angle sensor lives on the ESP32 board itself

## Pin mapping

| ESP32 pin | Function |
|-----------|----------|
| GPIO21 | I2C SDA — LCD |
| GPIO22 | I2C SCL — LCD |
| GPIO16 | Serial2 RX — angle data in, from the Wemos D1 mini's TX |
| GPIO17 | Serial2 TX — unused (the link is one-way, Wemos never listens) |
| GPIO25 | Motor 1 PWM output (no encoder — tracks motor 2's PWM + arm-angle trim) |
| GPIO26 | Motor 2 PWM output (closed loop, driven by the encoder below) |
| GPIO4  | Hall sensor / encoder pulse input (interrupt) — measures motor 2's shaft |
| GPIO34 | Motor 1 manual override potentiometer (ADC1-only pin) |
| GPIO35 | Motor 2 potentiometer — manual override PWM, or target speed setpoint in auto mode (ADC1-only pin) |
| GPIO13 | Button: LEFT |
| GPIO27 | Button: RIGHT |
| GPIO32 | Button: UP |
| GPIO33 | Button: DOWN |
| GPIO14 | Button: SELECT |

Potentiometers are on ADC1-only pins deliberately — ADC2 conflicts with Wi-Fi on the ESP32, so
manual-override reads stay reliable even if Wi-Fi is ever enabled. Any interrupt-capable GPIO
works for the encoder; GPIO4 is just what's wired today.

## Wemos D1 mini angle-sensor node

Firmware: [`software/WemosAngleSensor/WemosAngleSensor.ino`](software/WemosAngleSensor/WemosAngleSensor.ino) —
**finalized**. Reads an AS5600 magnetic angle sensor continuously and sends a line (e.g.
`"123.4\n"`) over its hardware serial only when the angle has moved more than 1 degree since
the last one sent. Purely one-way (Wemos -> ESP32), 9600 baud — nothing is requested or read
back, and the ESP32 side (`software/spool2spool-esp32`) just listens on Serial2 (GPIO16) and
keeps the last value it received.

| Wemos pin | Function |
|-----------|----------|
| D1 (GPIO5) | AS5600 SCL |
| D2 (GPIO4) | AS5600 SDA |
| TX | Data out, to the ESP32's GPIO16 (Serial2 RX) |
| 5V | Power in, from the ESP32 devkit's 5V/VIN pin |
| 3V3 | Powers the AS5600 — not the incoming 5V, keeps the sensor on the same 3.3V rail as the I2C logic |
| GND | Common ground, shared with the ESP32 and the cable shield |

The AS5600's DIR pin ties to GND (or VCC — either works, just don't leave it floating); GPO is
unused since only I2C is needed here.

Cable: simple 4-conductor wire for the ~1.5m run to the arm — one conductor for data, one for
power, two for GND (data GND and power GND, or just double up one GND conductor for lower
resistance). No twisted pair or shielding in use — not needed here, since a slow (9600 baud),
one-way UART link tolerates a plain cable run far better than I2C ever did over this distance,
which is why the angle sensor moved off I2C entirely in this architecture.

## Libraries

- `Wire.h` — built in
- `LiquidCrystal_I2C` — install via the Arduino Library Manager ("LiquidCrystal I2C"); needs
  a `begin(cols, rows)` variant, e.g. the DFRobot/Marco Schwartz-style fork most Library
  Manager searches return (the WARNING about it claiming "all architectures" is expected and
  harmless on ESP32)
- `AS5600` by Rob Tillaart — install via the Arduino Library Manager, needed only for the
  **Wemos** sketch, not the ESP32 sketch

## Building / flashing

**ESP32 main controller**: open
[`software/spool2spool-esp32/spool2spool-esp32.ino`](software/spool2spool-esp32/spool2spool-esp32.ino)
in the Arduino IDE, select an ESP32 Dev Module board (arduino-esp32 core 3.x — needed for
`analogWrite()` support), install `LiquidCrystal_I2C`, and upload.

**Wemos angle-sensor node**: open
[`software/WemosAngleSensor/WemosAngleSensor.ino`](software/WemosAngleSensor/WemosAngleSensor.ino),
select a Wemos D1 mini (ESP8266) board, install `AS5600`, and upload.

## Configuration

A handful of constants at the top of `spool2spool-esp32.ino` are meant to be tuned on the bench:

- `targetSpeedMPM` — motor 2's target speed in m/min, set continuously from pot 2 while in
  auto mode (see "What it does" above); the value at declaration is just a fallback default
- `Kp` — motor 2's proportional gain (speed loop); raised from an initial 0.5 (too slow, took
  ~20s to close a full-range error) to 3.0 — still just a starting guess, keep tuning on the
  bench, some overshoot is acceptable
- `spoolDiameterMM` — used to convert `targetSpeedMPM` to a target RPM for motor 2, and for
  the m/min display; live-adjustable via UP/DOWN, does not persist across power cycles
- `targetAngleDeg` — motor 1's target arm angle, live-adjustable via UP/DOWN; starting value
  is a placeholder
- `KpAngle` — motor 1's proportional gain for the angle trim; kept deliberately small since
  this is meant to be a fine correction, not the primary driver
- `angleTrimLimitPWM` — caps how far the angle trim can push motor 1 away from motor 2's PWM
- `angleMin` / `angleMax` — the arm's real angle range, placeholders until the real range of
  motion is measured; also clamps `targetAngleDeg` and the incoming `armAngle` reading
- `0x27` (LCD I2C address, in the `lcd()` constructor) — change to `0x3F` if the display stays
  blank, or run an I2C scanner sketch to confirm

On the Wemos side, `changeThresholdDeg` in `WemosAngleSensor.ino` sets how far the angle has to
move before a new reading is sent.

## TODO

Firmware:

- [ ] Tune `Kp` (motor 2 speed loop) against the real motor/load once motor 2 is running
- [ ] Tune `KpAngle` and `angleTrimLimitPWM` (motor 1's angle trim) on the bench
- [ ] Set `angleMin`/`angleMax` from the arm's actual range of motion, and pick a sensible
      starting `targetAngleDeg` within that range
- [ ] Confirm the AS5600's mounting orientation on the arm matches the sign/direction expected
      by `angleMin`/`angleMax` (`setDirection()` in `WemosAngleSensor.ino` flips it if needed)
- [ ] Button debounce is edge-detection only, no time-based debounce yet
- [ ] Both motors' correction loops run once per second; may need a shorter interval if RPM or
      angle swings under real load correct too slowly
- [ ] Option: give motor 1 its own encoder for real closed-loop RPM feedback, instead of
      tracking motor 2's PWM + angle trim. A free interrupt-capable GPIO is available for
      this (e.g. GPIO18) — same pattern as the existing encoder (`INPUT_PULLUP` +
      `attachInterrupt(..., FALLING)`). Not started yet.
- [ ] Real compile verification of `spool2spool-esp32.ino` via arduino-cli is still outstanding —
      blocked so far by a network policy restriction, not yet re-attempted

Hardware:

- [ ] Design a housing for the angle sensor (AS5600 + Wemos D1 mini together)
- [ ] Design a holder for the pulse counter (hall sensor)
- [ ] Design the overall case (ESP-WROOM-32 + I2C LCD + 5 buttons)
- [ ] Pick up a piece of DIN rail (or two) — 2026-09-15
- [ ] Verify the BTS7960 modules' logic inputs accept 3.3V from the ESP32 directly
