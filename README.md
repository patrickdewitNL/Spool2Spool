# Spool2Spool Controller

Dual-motor boom/tension controller: an ESP-WROOM-32 devkit driving two 24V geared DC motors
through BTS7960/IBT-2 drivers, with an I2C LCD for local readout/control and a separate
Wemos D1 mini node reading the boom angle right at the pivot.

## What it does

- **Motor 1** runs at a constant target speed, closed loop, using pulse feedback from a
  hall sensor/encoder on its shaft. A proportional controller corrects the PWM duty once per
  second. Speed is displayed as line speed in m/min, computed from the measured RPM and the
  spool diameter (adjustable live from the buttons).
- **Motor 2** follows the boom's angle. The angle itself is measured by an AS5600 magnetic
  sensor mounted right at the boom pivot, read by a separate Wemos D1 mini, and pushed to the
  ESP32 over a one-way serial link — see
  [Wemos D1 mini angle-sensor node](#wemos-d1-mini-angle-sensor-node) below.
- Both motors support **manual override**, driving speed directly from a potentiometer instead
  of the automatic logic.
- Both motor outputs stay at **zero on power-up** until SELECT is pressed once.
- Local UI: LEFT/RIGHT toggle manual/auto mode per motor, UP/DOWN adjust the spool diameter
  used for the speed readout, SELECT starts the system, and the 20x4 I2C LCD shows an
  "EMI Twente" splash screen at boot, a "press SELECT to start" prompt, then live speed/mode1
  on the top two lines and angle/mode2 on the bottom two.

## Hardware

- ESP-WROOM-32 devkit (30-pin), 3.3V logic throughout
- 20x4 I2C LCD (PCF8574-based backpack, address 0x27 or 0x3F depending on the module)
- 5 discrete pushbuttons (LEFT/RIGHT/UP/DOWN/SELECT), each to GND, using the ESP32's internal
  pull-ups — no external resistors, no resistor-ladder shield
- 2x 24V geared DC motors, each driven through a BTS7960/IBT-2 module (PWM speed control only,
  R_EN/L_EN tied high in hardware, no direction control needed)
- Hall sensor/encoder on motor 1's shaft, 1 pulse per revolution, interrupt driven
- 2x potentiometers for manual speed override, one per motor
- A separate **Wemos D1 mini + AS5600** node mounted at the boom pivot, sending the angle over
  a serial link (see below) — no angle sensor lives on the ESP32 board itself

## Pin mapping

| ESP32 pin | Function |
|-----------|----------|
| GPIO21 | I2C SDA — LCD |
| GPIO22 | I2C SCL — LCD |
| GPIO16 | Serial2 RX — angle data in, from the Wemos D1 mini's TX |
| GPIO17 | Serial2 TX — unused (the link is one-way, Wemos never listens) |
| GPIO25 | Motor 1 PWM output |
| GPIO26 | Motor 2 PWM output |
| GPIO4  | Hall sensor / encoder pulse input (interrupt) |
| GPIO34 | Motor 1 manual override potentiometer (ADC1-only pin) |
| GPIO35 | Motor 2 manual override potentiometer (ADC1-only pin) |
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

Cable: simple 4-conductor wire for the ~1.5m run to the boom — one conductor for data, one for
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

- `targetRPM` — motor 1's target speed
- `Kp` — motor 1's proportional gain, untested starting guess
- `spoolDiameterMM` — motor 1 spool diameter used for the m/min readout (also adjustable live via
  UP/DOWN; does not persist across power cycles)
- `angleMin` / `angleMax` — motor 2's boom angle range, placeholders until the real range of
  motion is measured
- `0x27` (LCD I2C address, in the `lcd()` constructor) — change to `0x3F` if the display stays
  blank, or run an I2C scanner sketch to confirm

On the Wemos side, `changeThresholdDeg` in `WemosAngleSensor.ino` sets how far the angle has to
move before a new reading is sent.

## TODO

Firmware:

- [ ] Tune `Kp` against the real motor/load once motor 1 is running
- [ ] Set `angleMin`/`angleMax` from the boom's actual range of motion
- [ ] Confirm the AS5600's mounting orientation on the boom matches the sign/direction expected
      by `angleMin`/`angleMax` (`setDirection()` in `WemosAngleSensor.ino` flips it if needed)
- [ ] Button debounce is edge-detection only, no time-based debounce yet
- [ ] Motor 1's correction loop runs once per second; may need a shorter interval if RPM swings
      under real load correct too slowly
- [ ] Real compile verification of `spool2spool-esp32.ino` via arduino-cli is still outstanding —
      blocked so far by a network policy restriction, not yet re-attempted

Hardware:

- [ ] Design a housing for the angle sensor (AS5600 + Wemos D1 mini together)
- [ ] Design a holder for the pulse counter (hall sensor)
- [ ] Design the overall case (ESP-WROOM-32 + I2C LCD + 5 buttons)
- [ ] Pick up a piece of DIN rail (or two) — 2026-09-15
- [ ] Verify the BTS7960 modules' logic inputs accept 3.3V from the ESP32 directly
