/*
  Spool2Spool Controller — Wemos D1 mini angle-sensor node
  Sits right next to the MPU6050 (short I2C run, avoids the noise/
  distance problems of running I2C 1.5m to the main controller) and
  answers angle requests from the ESP-WROOM-32 main controller over a
  bidirectional serial link. The ESP32 is the master: it decides when
  to ask, this board only ever responds.

  Wiring:
  - MPU6050 SDA -> D2 (GPIO4)
  - MPU6050 SCL -> D1 (GPIO5)
  - MPU6050 VCC -> this board's 3V3 pin (not the incoming 5V feed —
    keeps the sensor on the same 3.3V rail as the I2C logic)
  - MPU6050 GND -> GND
  - RX/TX (hardware UART) -> ESP32 TX/RX
  - 5V / GND -> power from the ESP32 side over the shared cable

  Protocol:
  - ESP32 sends a single request byte: 'A'
  - This board replies with the current boom angle as ASCII text
    followed by a newline, e.g. "18.3\n"
  - Anything else received is ignored, so noise on the line can't
    trigger a spurious reply

  Library needed:
  - MPU6050_light   (install via Library Manager, search "MPU6050_light" by rfetick —
    same one used on the main controller)
*/

#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

const char requestByte = 'A';
const long serialBaud = 9600;  // kept modest for noise margin over the long cable run

void setup() {
  Serial.begin(serialBaud);

  Wire.begin(D2, D1);  // SDA, SCL

  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("MPU6050 error");  // only visible if watched directly over USB during bench setup
    delay(1000);
    status = mpu.begin();
  }

  delay(500);
  mpu.calcOffsets();  // board must be still and level during this step
}

void loop() {
  mpu.update();  // keep the filter's time-integration fresh regardless of when a request arrives

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == requestByte) {
      float boomAngle = mpu.getAngleY();  // check this matches the physical mounting, same as the main sketch
      Serial.println(boomAngle, 1);
    }
    // anything else received is ignored
  }
}
