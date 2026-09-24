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
  (see above). The 20x4 I2C LCD otherwise shows a "press SELECT to start" prompt, then live speed/mode2 on the top two
  lines and angle/mode1 on the bottom two.

## Hardware

- ESP-WROOM-32 devkit (30-pin), 3.3V logic throughout
- 20x4 I2C LCD (PCF8574-based backpack, address 0x27)
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

## Machine Directive / CE compliance

This has two motorized moving parts (the arm and the spools) that can pinch or entangle, which
puts it in scope of the Machine Directive (2006/42/EC — being replaced by Machinery Regulation
(EU) 2023/1230, applicable from January 2027) if it's ever placed on the market rather than kept
as a one-off bench setup. It is **not** currently compliant. The main gap:

- **No emergency stop.** EN ISO 13850 / EN 60204-1 §9.2.5.4 require a readily-accessible
  emergency-stop device (red mushroom-head pushbutton, yellow background, latching/pull-to-release)
  on any machine with a hazard like this. Critically, it must cut power to both motors **in
  hardware** — wired in series with the 24V supply to the BTS7960 drivers (or through a safety
  relay/contactor), not just wired into a GPIO for the firmware to react to. A software-only stop
  (e.g. an E-stop button read like the existing LEFT/RIGHT/UP/DOWN/SELECT buttons, forcing
  `motor1PWM`/`motor2PWM` to 0) doesn't satisfy this: it stops working the moment the ESP32 hangs,
  crashes, is mid-OTA-update, or has a firmware bug — exactly when a hardware stop is needed most.
- This should be a **Category 0 stop** per EN 60204-1 §9.2.2 (immediate removal of power to the
  actuators) — simplest to implement here since these are plain DC motors with no regenerative
  braking or controlled-deceleration requirement.
- After an E-stop trip, the machine must **not** restart on its own when the button is released —
  it needs a deliberate manual reset (e.g. releasing the latch, then pressing SELECT again to
  restart, same as the existing power-up interlock already does).
- The E-stop's own monitoring/status wiring, if any, is the one case in this project's wiring
  where **orange** (EN 60204-1's color for circuits that stay energized with the main switch off)
  would actually apply, rather than the blue used for the ordinary control signals in the tables
  below.

Beyond the E-stop, a full CE Declaration of Conformity would also need a documented risk
assessment (EN ISO 12100) covering the arm/spool pinch points and the other Annex I essential
health and safety requirements — worth doing before this leaves the bench, not just the E-stop
wiring in isolation.

## Pin mapping

| ESP32 pin | Function | Cable color in cabinet | Suggested color (EN 60204-1) |
|-----------|----------|-------------|-------------|
| GPIO21 | I2C SDA — LCD | yellow | blue |
| GPIO22 | I2C SCL — LCD | green  | blue |
| GPIO16 | Serial2 RX — angle data in, from the Wemos D1 mini's TX | brown | blue |
| GPIO17 | Serial2 TX — connected to Wemos, not used |  | blue |
| GPIO25 | Motor 1 PWM output (no encoder — tracks motor 2's PWM + arm-angle trim) |  | blue |
| GPIO26 | Motor 2 PWM output (closed loop, driven by the encoder below) |  | blue |
| GPIO4  | Hall sensor / encoder pulse input (interrupt) — measures motor 2's shaft |  | blue |
| GPIO34 | Motor 1 manual override potentiometer (ADC1-only pin) |  | blue |
| GPIO35 | Motor 2 potentiometer — manual override PWM, or target speed setpoint in auto mode (ADC1-only pin) |  | blue |
| GPIO13 | Button: LEFT |  | blue |
| GPIO27 | Button: RIGHT |  | blue |
| GPIO32 | Button: UP |  | blue |
| GPIO33 | Button: DOWN |  | blue |
| GPIO14 | Button: SELECT |  | blue |

Every row above is a DC control-circuit conductor (logic-level signal, not a power feed), which
is why EN 60204-1 puts all of them under the same color — **blue**. The standard reserves other
colors for different roles: **black** for DC/AC power circuit conductors (the +3.3V/5V/GND
supply rails, not any row in this table), **green/yellow** exclusively for the protective earth
(PE) conductor bonded to the enclosure/frame (distinct from logic GND — see the note below), and
**orange** specifically for circuits that must stay energized when the main isolator is off, e.g.
an emergency-stop monitoring loop (see [Machine Directive / CE compliance](#machine-directive--ce-compliance)
below).

Red is always + 3.3V, Black is always GND. Blue is used for 5V from the ESP. Note this cabinet
convention doesn't itself follow EN 60204-1 (which reserves black for power circuits generally,
not GND specifically, and blue for DC control circuits, not a 5V supply feed) — it predates
adding the column above and hasn't been reconciled with it yet.



## Wemos D1 mini angle-sensor node

Firmware: [`software/WemosAngleSensor/WemosAngleSensor.ino`](software/WemosAngleSensor/WemosAngleSensor.ino) —
**finalized**. Reads an AS5600 magnetic angle sensor continuously and sends a line (e.g.
`"123.4\n"`) over its hardware serial only when the angle has moved more than 1 degree since
the last one sent. Purely one-way (Wemos -> ESP32), 9600 baud — nothing is requested or read
back, and the ESP32 side (`software/spool2spool-esp32`) just listens on Serial2 (GPIO16) and
keeps the last value it received.

| Wemos pin | Function | Cable color | Suggested color (EN 60204-1) |
|-----------|----------|-------------|-------------|
| D1 (GPIO5) | AS5600 SCL |  | blue (DC control circuit) |
| D2 (GPIO4) | AS5600 SDA |  | blue (DC control circuit) |
| TX | Data out, to the ESP32's GPIO16 (Serial2 RX) |  | blue (DC control circuit) |
| 5V | Power in, from the ESP32 devkit's 5V/VIN pin |  | black (DC power circuit) |
| 3V3 | Powers the AS5600 — not the incoming 5V, keeps the sensor on the same 3.3V rail as the I2C logic |  | black (DC power circuit) |
| GND | Common ground, shared with the ESP32 and the cable shield |  | black (power circuit return — only use green/yellow if this conductor is actually bonded to protective earth, not for a floating logic GND) |

The AS5600's DIR pin ties to GND (or VCC — either works, just don't leave it floating); GPO is
unused since only I2C is needed here.

Cable: simple 4-conductor wire for the ~1.5m run to the arm — one conductor for data, one for
power, one for GND, 1x TX one time RX.

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

Hardware / compliance:

- [ ] Add a hardware emergency stop (Category 0, EN ISO 13850 / EN 60204-1 §9.2.5.4) wired
      directly in series with the 24V supply to both BTS7960 drivers, not just a GPIO the
      firmware reads — see [Machine Directive / CE compliance](#machine-directive--ce-compliance)
- [ ] Document a risk assessment (EN ISO 12100) covering the arm/spool pinch points before this
      leaves the bench

