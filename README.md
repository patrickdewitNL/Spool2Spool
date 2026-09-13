# Spool2Spool Controller

Arduino Uno firmware for a dual-motor boom/tension controller, driving two 24V geared DC
motors through BTS7960/IBT-2 drivers and a DFRobot LCD Keypad Shield for local control and
readout.

## What it does

- **Motor 1** runs at a constant target speed, closed loop, using pulse feedback from a
  hall sensor/encoder on its shaft. A proportional controller corrects the PWM duty once per
  second. Speed is displayed as line speed in m/min, computed from the measured RPM and the
  spool diameter (adjustable live from the shield).
- **Motor 2** follows the boom's angle, read from an MPU6050 tilt sensor over I2C. Until the
  sensor is physically wired up, the angle is simulated with a sine sweep so the mapping logic
  and display can be tested without the hardware present.
- Both motors support **manual override**, driving speed directly from a potentiometer instead
  of the automatic logic.
- Both motor outputs stay at **zero on power-up** until SELECT is pressed once on the shield.
- The **DFRobot LCD Keypad Shield** provides local UI: LEFT/RIGHT toggle manual/auto mode per
  motor, UP/DOWN adjust the spool diameter used for the speed readout, SELECT starts the system,
  and the 16x2 LCD shows an "EMI Twente" splash screen at boot, a "press SELECT to start" prompt,
  then live speed/angle/mode.

## Hardware

- Arduino Uno
- DFRobot LCD Keypad Shield (16x2 LCD, 5 buttons on a single analog pin, parallel LCD interface)
- 2x 24V geared DC motors, each driven through a BTS7960/IBT-2 module (PWM speed control only,
  R_EN/L_EN tied high in hardware, no direction control needed)
- Hall sensor/encoder on motor 1's shaft, 1 pulse per revolution, interrupt driven
- MPU6050 accelerometer/gyro, bolted to the boom body as a tilt/inclinometer (I2C) — **not yet
  physically wired up**; stubbed out behind the `USE_MPU6050` compile-time flag
- 2x potentiometers for manual speed override, one per motor

## Pin mapping

The LCD shield occupies pins 4, 5, 6, 7, 8, 9, 10 (parallel LCD interface + backlight) and A0
(button ladder, analog read).

| Pin | Function |
|-----|----------|
| 2   | Hall sensor / encoder pulse input (INT0, hardware interrupt) |
| 3   | Motor 1 PWM output |
| 11  | Motor 2 PWM output |
| A2  | Motor 1 manual override potentiometer |
| A3  | Motor 2 manual override potentiometer |
| A4 / A5 | I2C (SDA/SCL) for the MPU6050, once connected |

Free/unused: pins 12, 13, A1.

## Libraries

- `LiquidCrystal.h` — built in
- `Wire.h` — built in
- [`MPU6050_light`](https://github.com/rfetick/MPU6050_light) by rfetick — install via the
  Arduino Library Manager, only needed once `USE_MPU6050` is set to `1`

## Building / flashing

Open `Spool2Spool/spool2spool.ino` in the Arduino IDE, select **Arduino Uno** as the board,
install the `MPU6050_light` library if `USE_MPU6050` is enabled, and upload.

## Configuration

A handful of constants at the top of the sketch are meant to be tuned on the bench:

- `targetRPM` — motor 1's target speed
- `Kp` — motor 1's proportional gain, untested starting guess
- `spoolDiameterMM` — motor 1 spool diameter used for the m/min readout (also adjustable live via
  the shield's UP/DOWN buttons; does not persist across power cycles)
- `angleMin` / `angleMax` — motor 2's boom angle range, placeholders until the real range of
  motion is measured
- `USE_MPU6050` — set to `1` once the MPU6050 is wired to A4/A5

## Known open items

- `Kp` needs tuning against the real motor/load once motor 1 is running
- Once the MPU6050 is wired up, confirm whether `getAngleX()` or `getAngleY()` matches the
  physical mounting orientation
- `angleMin`/`angleMax` need to be set from the boom's actual range of motion
- LCD shield button debounce is edge-detection only (no time-based debounce yet)
- Motor 1's correction loop runs once per second; may need a shorter interval if RPM swings
  under real load correct too slowly
