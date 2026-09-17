/*
  Spool2Spool Controller — Wemos D1 mini angle-sensor node (AS5600)
  Reads the AS5600 continuously and actively pushes a new angle reading
  over Serial whenever it changes by more than 1 degree since the last
  one sent — no request/response, no polling from the receiving end,
  this board sends updates on its own initiative.

  Swapped from an MPU6050-based request/response node to this AS5600
  active-push one — see git history for the previous version if the
  MPU6050 approach is still wanted.

  Wiring:
  - AS5600 VCC -> this board's 3V3 pin (most AS5600 breakouts handle
    3.3-5V fine; 3.3V keeps everything on one logic rail)
  - AS5600 GND -> GND
  - AS5600 SCL -> D1 (GPIO5)
  - AS5600 SDA -> D2 (GPIO4)
  - AS5600 DIR -> GND (hardware direction pin — tie to GND or VCC,
    doesn't matter which, just don't leave it floating)
  - AS5600 GPO -> not connected (I2C only)
  - RX/TX (hardware UART) -> ESP32 TX/RX
  - 5V / GND -> power from the ESP32 side over the shared cable

  Protocol:
  - Sends a line like "123.4\n" whenever the angle has moved more than
    1 degree since the last line sent — nothing is sent otherwise
  - Purely one-way (this board -> ESP32). If the ESP32 ever needs to
    ask for a reading on demand instead of just listening for pushes,
    that's a different scheme than this one.

  Library needed:
  - AS5600 by Rob Tillaart (install via Library Manager, search "AS5600")
*/

#include <Wire.h>
#include "AS5600.h"

AS5600 as5600;  // standard AS5600, not the AS5600L variant — swap the class if yours is an L

const float changeThresholdDeg = 1.0;
const long serialBaud = 9600;  // kept modest for noise margin over the long cable run

float lastSentAngle = -1000;  // far outside any real angle, guarantees the first reading gets sent

void setup() {
  Serial.begin(serialBaud);
  delay(500);

  Wire.begin(D2, D1);  // SDA, SCL

  as5600.begin();
  as5600.setDirection(AS5600_CLOCK_WISE);  // arbitrary but explicit; just needs to stay consistent

  if (!as5600.isConnected()) {
    Serial.println("AS5600 NOT connected - check wiring");  // only visible over USB during bench setup
  }
}

void loop() {
  float angle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;

  // note: a change spanning the 0/360 wrap point reads as a large jump here, which still
  // correctly triggers a send — it just doesn't take the shorter way around the circle
  // into account. Not worth the extra complexity unless that turns out to matter in practice.
  if (abs(angle - lastSentAngle) > changeThresholdDeg) {
    Serial.println(angle, 1);
    lastSentAngle = angle;
  }

  delay(50);  // plenty fast to catch a 1-degree change on a slow-moving shaft, without hammering I2C
}
