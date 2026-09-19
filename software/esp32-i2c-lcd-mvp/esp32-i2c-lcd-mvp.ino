/*
  MVP: ESP32 + I2C 20x4 LCD bring-up test
  Minimal standalone sketch to verify an I2C 20x4 character LCD (PCF8574-based backpack)
  works on an ESP32 devkit, before wiring it into the full spool2spool logic
  (software/spool2spool-esp32). Boots, scans the I2C bus for diagnostics, shows a splash
  screen, then a live uptime counter on all 4 rows so you can confirm every row/column works.

  Wiring (30-pin ESP32 devkit):
  - LCD SDA -> GPIO21
  - LCD SCL -> GPIO22
  - LCD VCC -> 5V        (most PCF8574 backpacks want 5V for the backlight/contrast even
                           though the I2C logic itself is 3.3V-tolerant; check your specific
                           backpack if the backlight looks dim on 3.3V)
  - LCD GND -> GND

  Libraries needed:
  - Wire.h                 (built in)
  - LiquidCrystal_I2C.h    (install via Library Manager, search "LiquidCrystal I2C" — needs
    a begin(cols, rows) variant, e.g. the one bundled with the DFRobot/Marco Schwartz-style
    fork most Library Manager searches return; default I2C address assumed 0x27 below,
    change it if your backpack uses 0x3F, or watch the I2C scan output at boot)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---- I2C pins ----
const int sdaPin = 21;
const int sclPin = 22;

// address 0x27 is the common default for PCF8574-based backpacks; some use 0x3F instead
LiquidCrystal_I2C lcd(0x27, 20, 4);

// scans all 7-bit I2C addresses and prints whatever ACKs — run once at boot so a wiring or
// address problem (wrong SDA/SCL, no pull-ups, wrong backpack address) shows up on Serial
// before we even try to talk to the LCD
void scanI2C() {
  Serial.println("I2C scan starting...");
  int found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  I2C device found at 0x");
      Serial.println(address, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  no I2C devices found -- check SDA/SCL wiring (GPIO21/22), backpack "
                    "power, and pull-ups");
  } else {
    Serial.print(found);
    Serial.println(" I2C device(s) found");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // gives the USB serial monitor time to connect before the first prints
  Serial.println("=== esp32-i2c-lcd-mvp booting ===");

  Wire.begin(sdaPin, sclPin);
  Serial.println("Wire.begin() done (SDA=GPIO21, SCL=GPIO22)");
  scanI2C();

  Serial.println("Calling lcd.begin(20, 4)...");
  lcd.begin(20, 4);
  Serial.println("lcd.begin() returned");
  lcd.backlight();
  Serial.println("lcd.backlight() called -- if the backlight LED isn't lit now, check power "
                  "to the backpack (5V vs 3.3V) before looking at I2C further");

  lcd.clear();
  lcd.setCursor(5, 0);  // (20 - 10) / 2, centers "EMI Twente" on a 20 column display
  lcd.print("EMI Twente");
  lcd.setCursor(2, 1);
  lcd.print("I2C LCD MVP test");
  delay(2000);
  lcd.clear();
}

void loop() {
  // fills all 4 rows so you can confirm every row and the full 20-column width work
  unsigned long seconds = millis() / 1000;

  char line[21];
  snprintf(line, sizeof(line), "Uptime: %lus", seconds);
  lcd.setCursor(0, 0);
  lcd.print("Row0: 01234567890123456");
  lcd.setCursor(0, 1);
  lcd.print(line);
  lcd.setCursor(0, 2);
  lcd.print("Row2: OK if visible");
  lcd.setCursor(0, 3);
  lcd.print("Row3: OK if visible");

  Serial.print("Uptime: ");
  Serial.print(seconds);
  Serial.println("s");

  delay(1000);
}
