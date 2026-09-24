/*
  Dual motor control, ESP32 (30-pin devkit) + I2C 20x4 LCD
  Ported from software/spool2spool-arduino/spool2spool-arduino.ino — same control
  logic and behavior, retargeted for the ESP32 + I2C LCD, with 5 discrete buttons
  replacing the DFRobot shield's analog button ladder (that ladder doesn't exist
  without the shield; the button/keypad approach was still open in the README TODO,
  discrete GPIOs is the simplest default and easy to swap for an I2C keypad later).

  - Motor 2 has the only encoder/hall sensor: closed loop constant speed control, target
    set live as a speed in m/min (converted to RPM via the spool diameter). Pot 2 sets this
    target speed continuously in auto mode, and drives the PWM directly (open loop) in
    manual mode -- same knob, same feel, different meaning depending on mode.
  - Motor 1 has no encoder of its own, so its baseline PWM continuously tracks motor 2's
    PWM (keeping both motors nominally speed-matched), fine-tuned by a small proportional
    correction based on the arm angle -- received over Serial2 from the Wemos D1 mini
    angle-sensor node (software/WemosAngleSensor — AS5600 next to the arm pivot) -- holding
    that angle at a live-adjustable target. The angle term is deliberately a small trim on
    top of the motor-2-tracking baseline, not the primary driver: the angle reading is
    noisy/unreliable right at startup, so a bad first reading can only nudge motor 1 a
    little, never throw it far from motor 2's PWM.
  - Buttons: LEFT/RIGHT toggle manual/auto mode per motor. SELECT starts the system, then
    (once started) cycles what UP/DOWN adjusts: spool diameter, then motor 1's target angle.
  - Both motor outputs stay at zero after power up until SELECT is pressed once
  - LCD shows an "EMI Twente" splash screen at boot, then a "press SELECT to start" prompt,
    then live speed/angle/mode

  Wiring (30-pin ESP32 devkit):
  - LCD SDA -> GPIO21, LCD SCL -> GPIO22 (default I2C — LCD is the only I2C device on this  groen/geel
    board now)
  - Serial2 RX -> GPIO16, from the Wemos D1 mini's TX (one-way link — this board's TX/GPIO17  
    is unused, the Wemos never listens)
  - motor1PwmPin -> GPIO25 (motor 1 driver RPWM)  paars 
  - motor2PwmPin -> GPIO26 (motor 2 driver RPWM) paars
  - encoderPin    -> GPIO4  (hall sensor / encoder, interrupt capable — any GPIO works on ESP32)  oranje
  - pot1Pin -> GPIO34, grijs
  - pot2Pin -> GPIO35 (ADC1-only pins — avoids ADC2/WiFi conflicts) grijs

  - buttons -> GPIO13 (LEFT), GPIO27 (RIGHT), GPIO32 (UP), GPIO33 (DOWN), GPIO14 (SELECT),
    each wired to GND through a pushbutton, using the internal pull-up (no external resistors)

  Libraries needed:
  - Wire.h                 (built in)
  - LiquidCrystal_I2C.h    (install via Library Manager, search "LiquidCrystal I2C" — needs
    a begin(cols, rows) variant, e.g. the one bundled with the DFRobot/Marco Schwartz-style
    fork most Library Manager searches return; default I2C address assumed 0x27 below,
    change if your backpack uses 0x3F, or run an I2C scanner sketch if the display stays blank)
  - WiFi.h, WebServer.h    (built in)
  - ElegantOTA.h           (install via Library Manager, search "ElegantOTA" by Ayush Sharma)

  Requires arduino-esp32 core 3.x — that's what gives analogWrite() direct PWM support here,
  same call signature as the original AVR code. Older cores need ledcWrite() instead.

  WiFi / OTA / web dashboard:
  - Once this board runs off an external 5V supply instead of USB, wifiSsid/wifiPassword below
    must be filled in before flashing -- that's the only way to reach it afterwards.
  - The web page at http://<device-ip>/ mirrors the LCD (speed, angle, motor %, modes) plus a
    live graph; firmware updates happen from the browser at http://<device-ip>/update
    (ElegantOTA), no Arduino IDE or USB needed.
  - The graph's history lives in the browser's localStorage, not on the ESP32 -- it only
    accumulates while that browser tab has been polling, resets if you clear site data, and
    isn't shared between browsers/devices viewing the page.
  - The dashboard's Chart.js is loaded from a CDN, so the *browser* needs internet access to
    render the graph (the ESP32 itself only needs to be on the same LAN, no internet required).
  - WiFi connection is best-effort and never blocks motor control: it times out after 10s at
    boot if not connected -- the motors run with or without WiFi.
  - AP fallback: if the configured network isn't reachable at boot, the device becomes its own
    WiFi access point (apSsid/apPassword below) instead of being unreachable -- connect a phone
    or laptop to it and the same dashboard/OTA page is at http://192.168.4.1 (ESP32's default
    AP address). This is the way to push a firmware fix out in the field with no WiFi around.
    It only kicks in once, at boot -- if it falls back to AP, it stays on AP until rebooted, it
    doesn't keep trying the configured network in the background afterwards.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "dashboard_html.h"

// ---- I2C LCD ----
// address 0x27 is the common default for PCF8574-based backpacks; some use 0x3F instead
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ---- WiFi / OTA / web dashboard ----
// fill these in before flashing -- see header comment above for what this enables. Not
// secret enough to warrant anything fancier than living here, this device never leaves a
// private network.
const char *wifiSsid = "VerkeerdVerbonden";
const char *wifiPassword = "welkomopzuiderzee663";
const unsigned long wifiConnectTimeoutMs = 10000;  // best-effort at boot, never blocks longer than this

// AP fallback -- if wifiSsid isn't reachable at boot, the device hosts this network itself
// instead, so the dashboard/OTA page is always reachable somehow. WPA2-protected since this
// AP exposes the /update firmware page to whoever joins it -- change the password below.
const char *apSsid = "Spool2Spool-Setup";
const char *apPassword = "spool2spool";  // WPA2 needs >= 8 chars

WebServer server(80);
bool otaReady = false;  // true once server.begin()/ElegantOTA.begin() have run (STA or AP)

// ---- Motor pins ----
const int motor1PwmPin = 25;  // motor 1, no encoder -- tracks motor 2's PWM, fine-tuned by arm angle
const int motor2PwmPin = 26;  // motor 2, closed loop RPM control (this motor has the encoder)

// ---- Encoder / hall pulse pin (motor 2 speed feedback -- the only encoder in this system) ----
const int encoderPin = 4;     // interrupt capable — any GPIO works on ESP32
volatile unsigned long pulseCount = 0;          // lifetime count, diagnostics only
volatile unsigned long lastPulseMicros = 0;     // micros() at the most recent pulse
volatile unsigned long pulsePeriodMicros = 0;   // time between the last two pulses
// no pulse for this long -> treat as stopped rather than let the RPM estimate decay forever
const unsigned long stalledTimeoutMicros = 10000000UL;  // 10s, ~2x the slowest expected pulse interval
unsigned long lastCalcTime = 0;
float currentRPM = 0;
float currentLinearSpeed = 0;  // m/min, derived from the smoothed RPM and spoolDiameterMM, for display only

// ---- Speed display smoothing ----
// currentRPM (used by the P-controller below) stays raw/unfiltered on purpose, so this only
// smooths what's shown on the LCD/serial, it doesn't change motor control response.
const int speedAvgSamples = 5;  // ~5 seconds of smoothing at the once-per-second update rate
float speedAvgBuffer[speedAvgSamples] = {0};
int speedAvgIndex = 0;
int speedAvgCount = 0;  // samples recorded so far, caps at speedAvgSamples

// ---- Potentiometers ----
const int pot1Pin = 34;  // motor 1 manual speed (ADC1-only pin)
const int pot2Pin = 35;  // motor 2: manual PWM in manual mode, target speed setpoint in
                          // auto mode -- same knob, different meaning (ADC1-only pin)

// ---- Buttons (discrete GPIOs, internal pull-up, pressed = LOW) ----
const int btnLeftPin   = 13;
const int btnRightPin  = 27;
const int btnUpPin     = 32;
const int btnDownPin   = 33;
const int btnSelectPin = 14;

// ---- Start interlock, both motor outputs stay at zero until SELECT is pressed once ----
bool started = false;

// ---- Mode state, true = manual, false = automatic ----
bool manualMode1 = false;
bool manualMode2 = false;

// ---- Motor 2 closed loop speed control -- target set live from pot2 in auto mode (see
// loop()), converted to RPM below using the spool diameter ----
float targetSpeedMPM = 5.0;             // m/min, set from pot2 every loop while in auto mode
const float speedMinMPM = 0.0;
const float speedMaxMPM = 10.0;
int motor2PWM = 0;                      // starting point, self adjusts -- overwritten in setup()
                                         // from pot2's actual reading, not a hardcoded guess
const float Kp = 4.0;                   // raised from 0.5 -- that crawled toward setpoint over
                                         // ~20s for a full-range error; some overshoot is fine,
                                         // keep raising if still slow, lower if it starts hunting

// ---- Motor 1's angle range -- adjust to match your arm's real min/max angle in degrees ----
const float angleMin = 0.0;
const float angleMax = 350.0;

// ---- Motor 1 tracks motor 2's PWM (no encoder of its own), fine-tuned by arm angle ----
float targetAngleDeg = 90.0;            // placeholder, adjust for your setup -- live-adjustable
const float angleStepDeg = 0.1;
const float KpAngle = 1.0;              // deliberately modest -- this is a fine trim, not the primary driver
const int angleTrimLimitPWM = 40;       // caps how far the angle trim can push motor 1 away from
                                         // motor 2's PWM, so a noisy/garbage angle reading (e.g.
                                         // before the Wemos's first line ever arrives) can't swing it far
int motor1PWM = 0;  // overwritten in setup() from pot1's actual reading, not a hardcoded guess

// ---- Spool, for converting RPM <-> line speed (motor 2's target and the display) ----
// UP/DOWN adjust this live, e.g. to match whatever spool is currently loaded
// NOTE: this does not persist across power cycles, resets to the placeholder below on reset
float spoolDiameterMM = 20.0;  // placeholder, adjust for the real spool on the bench
const float spoolDiameterStepMM = 1.0;
const float spoolDiameterMinMM = 5.0;
const float spoolDiameterMaxMM = 300.0;

// ---- cycles which of the 2 values above UP/DOWN currently adjusts -- SELECT advances this
// once the system has started (before that, SELECT starts it instead). Motor 2's target
// speed isn't in this cycle -- pot2 sets that continuously in auto mode instead. ----
int adjustTarget = 0;  // 0 = spool diameter, 1 = motor 1 target angle
unsigned long adjustDisplayUntil = 0;  // while in the future, LCD shows the adjusted value instead of normal telemetry
const unsigned long adjustDisplayDuration = 1500;

// ---- Arm angle, received from the Wemos D1 mini angle-sensor node over Serial2 ----
// The Wemos only sends a new line when the angle has moved more than 1 degree (see
// software/WemosAngleSensor), not on a fixed schedule — this just holds the last value
// received until the next line comes in.
float armAngle = 0.0;
String angleLineBuffer = "";

// ---- LCD refresh timing ----
unsigned long lastLcdUpdate = 0;

// ---- Button reading ----
// same return codes as the original shield version (0=right,1=up,2=down,3=left,4=select,
// -1=none), so the rest of the button-handling logic below didn't need to change at all
int readLCDButton() {
  if (digitalRead(btnSelectPin) == LOW) return 4;
  if (digitalRead(btnLeftPin)   == LOW) return 3;
  if (digitalRead(btnDownPin)   == LOW) return 2;
  if (digitalRead(btnUpPin)     == LOW) return 1;
  if (digitalRead(btnRightPin)  == LOW) return 0;
  return -1;
}
int lastButton = -1;

void countPulse() {
  unsigned long now = micros();
  pulsePeriodMicros = now - lastPulseMicros;
  lastPulseMicros = now;
  pulseCount++;
}

// pads/truncates to exactly 20 chars so leftover characters from a longer previous
// line (e.g. more digits) never linger on screen
void lcdPrintLine(int row, const char *text) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%-20s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// left text at column 0, right text flush against column 20 (e.g. a live reading on the
// left, its setpoint right-aligned) -- if both together don't fit, right still gets at
// least 1 space of separation and just gets clipped by lcdPrintLine's own 20-char truncation
void lcdPrintLineRJ(int row, const char *left, const char *right) {
  char buf[21];
  int spaces = 20 - (int)strlen(left) - (int)strlen(right);
  if (spaces < 1) spaces = 1;
  snprintf(buf, sizeof(buf), "%s%*s", left, spaces + (int)strlen(right), right);
  lcdPrintLine(row, buf);
}

// non-blocking: drains whatever's waiting on Serial2 and updates armAngle once a full
// line has arrived. Never waits for more data, so this can't stall the main loop.
void updateArmAngleFromSerial() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      if (angleLineBuffer.length() > 0) {
        armAngle = angleLineBuffer.toFloat();
      }
      angleLineBuffer = "";
    } else if (c != '\r') {
      angleLineBuffer += c;
    }
  }
}

void handleRoot() {
  server.send_P(200, "text/html", dashboardHtml);
}

void handleData() {
  char json[256];
  int m1Percent = (motor1PWM * 100 + 127) / 255;
  int m2Percent = (motor2PWM * 100 + 127) / 255;
  snprintf(json, sizeof(json),
           "{\"speed\":%.2f,\"targetSpeed\":%.2f,\"angle\":%.2f,\"targetAngle\":%.2f,"
           "\"m1Percent\":%d,\"m2Percent\":%d,\"mode1\":\"%s\",\"mode2\":\"%s\","
           "\"started\":%s}",
           currentLinearSpeed, targetSpeedMPM, armAngle, targetAngleDeg, m1Percent, m2Percent,
           manualMode1 ? "MAN" : "AUTO", manualMode2 ? "MAN" : "AUTO",
           started ? "true" : "false");
  server.send(200, "application/json", json);
}

// registers the dashboard/OTA routes and starts the web server -- shared by both the normal
// (STA) and fallback (AP) paths below, since the page itself doesn't care which interface it's
// reachable on
void bindRoutesAndStartServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  ElegantOTA.begin(&server);
  server.begin();
  otaReady = true;
}

// called once WiFi connects, either at boot or from a later background retry (see loop())
void startWebServices() {
  Serial.print("WiFi connected, dashboard/OTA at http://");
  Serial.println(WiFi.localIP());
  bindRoutesAndStartServer();
}

// no configured network reachable -- become one instead, so the dashboard/OTA page (and a way
// to push a firmware fix) is always reachable directly from a phone/laptop, even in the field
// with no WiFi around
void startFallbackAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, apPassword);
  Serial.print("WiFi not found -- started fallback AP \"");
  Serial.print(apSsid);
  Serial.print("\", dashboard/OTA at http://");
  Serial.println(WiFi.softAPIP());
  bindRoutesAndStartServer();
}

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
    Serial.println("  no I2C devices found -- check SDA/SCL wiring (GPIO21/22) and that the "
                    "LCD backpack has power");
  } else {
    Serial.print(found);
    Serial.println(" I2C device(s) found");
  }
}

void setup() {
  Serial.begin(115200);  // matches the ESP32's default ROM bootloader baud -- set the Serial
                          // Monitor to 115200 too, or these prints come out as garbage
  delay(500);  // gives the USB serial monitor time to connect before the first prints
  Serial.println("=== spool2spool-esp32 booting ===");

  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX from the Wemos; TX (GPIO17) is unused
  Serial.println("Serial2 (Wemos angle link) initialized");

  pinMode(btnLeftPin, INPUT_PULLUP);
  pinMode(btnRightPin, INPUT_PULLUP);
  pinMode(btnUpPin, INPUT_PULLUP);
  pinMode(btnDownPin, INPUT_PULLUP);
  pinMode(btnSelectPin, INPUT_PULLUP);
  Serial.println("Button pins configured (INPUT_PULLUP) -- note: with no button wired to a "
                  "pin, it just reads HIGH/not-pressed forever, that's expected");

  Wire.begin();  // default ESP32 I2C pins: SDA=21, SCL=22
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
  lcd.setCursor(1, 2);  // (20 - 17) / 2, rounded down, centers "CoatyMac Coatface" on the third line
  lcd.print("CoatyMac Coatface");
  Serial.println("Splash text sent to LCD");
  delay(2000);
  lcd.clear();

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, FALLING);

  pinMode(motor1PwmPin, OUTPUT);
  pinMode(motor2PwmPin, OUTPUT);
  analogWrite(motor1PwmPin, 0);
  analogWrite(motor2PwmPin, 0);

  // seed the auto-mode PID starting points from wherever the pots actually are right now,
  // instead of a hardcoded guess -- same inverted mapping as manual mode uses
  motor1PWM = map(analogRead(pot1Pin), 0, 4095, 255, 0);
  motor2PWM = map(analogRead(pot2Pin), 0, 4095, 255, 0);

  lastCalcTime = millis();

  // ---- WiFi / OTA / dashboard: best-effort, bounded wait -- a missing/slow network must
  // never hold up motor control, so this gives up after wifiConnectTimeoutMs and falls back
  // to hosting its own AP instead of blocking here indefinitely ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < wifiConnectTimeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    startWebServices();
  } else {
    startFallbackAP();
  }

  Serial.println("=== setup() complete, entering loop() ===");
}

unsigned long lastWaitingDebug = 0;

void loop() {
  updateArmAngleFromSerial();  // non-blocking; keeps armAngle fresh whenever the Wemos sends

  // ---- dashboard/OTA request handling -- setup() always leaves the server running, on
  // either the configured network or the fallback AP, so this is unconditional. Reachable
  // even before SELECT is pressed, deliberately ahead of the !started early-return below. ----
  server.handleClient();
  ElegantOTA.loop();

  // ---- both outputs stay at zero until SELECT has been pressed once ----
  if (!started) {
    int startBtn = readLCDButton();
    if (startBtn == 4) {  // SELECT
      started = true;
      lastButton = startBtn;
      lcd.clear();
      Serial.println("SELECT pressed -- started");
    } else {
      analogWrite(motor1PwmPin, 0);
      analogWrite(motor2PwmPin, 0);
      if (millis() - lastLcdUpdate > 250) {
        lcdPrintLine(1, "Press SELECT to");
        lcdPrintLine(2, "start");
        lastLcdUpdate = millis();
      }
      if (millis() - lastWaitingDebug > 2000) {
        Serial.println("Waiting for SELECT (GPIO14 pulled LOW) -- if buttons aren't wired "
                        "yet, this is expected and will just sit here");
        lastWaitingDebug = millis();
      }
      return;  // skip mode/angle/motor logic entirely until started
    }
  }

  // ---- read buttons, act on new press only ----
  int btn = readLCDButton();
  if (btn != -1 && btn != lastButton) {
    if (btn == 3) manualMode1 = !manualMode1;  // LEFT toggles motor 1 mode
    if (btn == 0) manualMode2 = !manualMode2;  // RIGHT toggles motor 2 mode

    if (btn == 4) {  // SELECT (after start): cycle which value UP/DOWN adjusts
      adjustTarget = (adjustTarget + 1) % 2;
      adjustDisplayUntil = millis() + adjustDisplayDuration;
    }

    if (btn == 1) {  // UP
      if (adjustTarget == 0) {
        spoolDiameterMM = min(spoolDiameterMM + spoolDiameterStepMM, spoolDiameterMaxMM);
      } else {
        targetAngleDeg = min(targetAngleDeg + angleStepDeg, angleMax);
      }
      adjustDisplayUntil = millis() + adjustDisplayDuration;
    }
    if (btn == 2) {  // DOWN
      if (adjustTarget == 0) {
        spoolDiameterMM = max(spoolDiameterMM - spoolDiameterStepMM, spoolDiameterMinMM);
      } else {
        targetAngleDeg = max(targetAngleDeg - angleStepDeg, angleMin);
      }
      adjustDisplayUntil = millis() + adjustDisplayDuration;
    }
  }
  lastButton = btn;

  // ---- motor 2: manual pot drives PWM directly, auto pot sets the target speed instead ----
  int potVal2 = analogRead(pot2Pin);
  if (manualMode2) {
    motor2PWM = map(potVal2, 0, 4095, 255, 0);  // inverted: potVal 0 = full, 4095 = stop. ESP32 ADC is 12-bit (0-4095), not 10-bit like AVR
    analogWrite(motor2PwmPin, motor2PWM);
  } else {
    // same inverted feel as manual mode: 0 = max target speed, 4095 = stop
    targetSpeedMPM = speedMaxMPM - (speedMaxMPM - speedMinMPM) * (potVal2 / 4095.0);
  }

  // ---- motor 1: manual pot override (auto = tracks motor 2's PWM + arm-angle trim, below) ----
  if (manualMode1) {
    int potVal1 = analogRead(pot1Pin);
    motor1PWM = map(potVal1, 0, 4095, 255, 0);  // inverted: potVal 0 = full, 4095 = stop. ESP32 ADC is 12-bit (0-4095), not 10-bit like AVR
    analogWrite(motor1PwmPin, motor1PWM);
  }

  if (millis() - lastCalcTime >= 1000) {
    noInterrupts();
    unsigned long period = pulsePeriodMicros;
    unsigned long lastPulse = lastPulseMicros;
    interrupts();

    unsigned long sinceLastPulse = micros() - lastPulse;

    if (lastPulse == 0 || sinceLastPulse > stalledTimeoutMicros) {
      // no pulse yet, or it's been way longer than a normal interval -> genuinely stopped
      currentRPM = 0;
    } else {
      // use whichever is longer: the last measured interval, or how long it's been since
      // that pulse — so the estimate keeps easing down every second if the shaft is
      // slowing, instead of freezing at the last (higher) reading until the next pulse
      unsigned long effectivePeriod = max(period, sinceLastPulse);
      currentRPM = 60000000.0 / effectivePeriod;  // change multiplier if more than 1 pulse per rev
    }

    // moving average over the last speedAvgSamples RPM readings, smooths the displayed
    // speed so it eases toward 0 instead of snapping there the instant pulses stop
    speedAvgBuffer[speedAvgIndex] = currentRPM;
    speedAvgIndex = (speedAvgIndex + 1) % speedAvgSamples;
    if (speedAvgCount < speedAvgSamples) speedAvgCount++;

    float rpmSum = 0;
    for (int i = 0; i < speedAvgCount; i++) rpmSum += speedAvgBuffer[i];
    float avgRPM = rpmSum / speedAvgCount;

    currentLinearSpeed = avgRPM * PI * (spoolDiameterMM / 1000.0);  // m/min, smoothed

    // ---- motor 2: closed loop, target speed converted to RPM via the spool diameter ----
    if (!manualMode2) {
      float targetRPM = targetSpeedMPM / (PI * (spoolDiameterMM / 1000.0));
      float error = targetRPM - currentRPM;
      motor2PWM += (int)(Kp * error / 10);
      motor2PWM = constrain(motor2PWM, 0, 255);
      analogWrite(motor2PwmPin, motor2PWM);
    }

    // ---- motor 1: no encoder -- track motor 2's PWM, trimmed by arm angle error ----
    if (!manualMode1) {
      if (motor2PWM == 0) {
        // motor 2 stopped (e.g. 0 m/min setpoint) -> motor 1 stops too, angle trim doesn't
        // apply here: a positive trim on top of 0 would otherwise still spin motor 1
        motor1PWM = 0;
      } else {
        float clampedAngle = constrain(armAngle, angleMin, angleMax);
        float angleError = targetAngleDeg - clampedAngle;
        int trim = constrain((int)(KpAngle * angleError), -angleTrimLimitPWM, angleTrimLimitPWM);
        motor1PWM = constrain(motor2PWM + trim, 0, 255);
      }
      analogWrite(motor1PwmPin, motor1PWM);
    }

    Serial.print("RPM: ");
    Serial.print(currentRPM);
    Serial.print(" | Speed(m/min): ");
    Serial.print(currentLinearSpeed);
    Serial.print(" | TargetSpeed(m/min): ");
    Serial.print(targetSpeedMPM);
    Serial.print(" | Motor2 PWM: ");
    Serial.print(motor2PWM);
    Serial.print(" | Mode2: ");
    Serial.print(manualMode2 ? "MAN" : "AUTO");
    Serial.print(" | Angle: ");
    Serial.print(armAngle);
    Serial.print(" | TargetAngle: ");
    Serial.print(targetAngleDeg);
    Serial.print(" | Motor1 PWM: ");
    Serial.print(motor1PWM);
    Serial.print(" | Mode1: ");
    Serial.print(manualMode1 ? "MAN" : "AUTO");
    Serial.print(" | Potval1: ");
    Serial.print(analogRead(pot1Pin));
    Serial.print(" | Potval2: ");
    Serial.println(analogRead(pot2Pin));

    lastCalcTime = millis();
  }

  // ---- LCD update, every 250ms so its readable, not flickering ----
  if (millis() - lastLcdUpdate > 250) {
    char line[21];
    if (millis() < adjustDisplayUntil) {
      if (adjustTarget == 0) {
        lcdPrintLine(0, "Spool diameter");
        snprintf(line, sizeof(line), "%.0fmm", spoolDiameterMM);
        lcdPrintLine(1, line);
      } else {
        lcdPrintLine(0, "Motor1 target angle");
        snprintf(line, sizeof(line), "%.1fdeg", targetAngleDeg);
        lcdPrintLine(1, line);
      }
      lcdPrintLine(2, "");
      lcdPrintLine(3, "");
    } else {
      char setpoint[11];
      // setpoints only mean anything in auto mode -- in manual mode the pot drives PWM
      // directly, there's no setpoint to show, so the line is just the live reading
      snprintf(line, sizeof(line), "Spd %.1fm/min", currentLinearSpeed);
      if (manualMode2) {
        // no setpoint in manual mode -- pot2 drives PWM directly, nothing to show at right
        lcdPrintLine(0, line);
      } else {
        snprintf(setpoint, sizeof(setpoint), "SP %-4.1f", targetSpeedMPM);
        lcdPrintLineRJ(0, line, setpoint);
      }

      snprintf(line, sizeof(line), "Ang %.1fdeg", armAngle);
      if (manualMode1) {
        // no setpoint in manual mode -- pot1 drives PWM directly, nothing to show at right
        lcdPrintLine(1, line);
      } else {
        snprintf(setpoint, sizeof(setpoint), "SP %-4.1f", targetAngleDeg);
        lcdPrintLineRJ(1, line, setpoint);
      }

      // both motors' mode fit on one line -- MAN/AUTO abbreviated to keep the pair short
      snprintf(line, sizeof(line), "M1:%-4s      M2:%-4s", manualMode1 ? "MAN" : "AUTO",
               manualMode2 ? "MAN" : "AUTO");
      lcdPrintLine(2, line);

      // actual drive level under the mode line, scaled from raw PWM (0-255) to 0-100%
      char pct1[5], pct2[5];
      snprintf(pct1, sizeof(pct1), "%d%%", (motor1PWM * 100 + 127) / 255);
      snprintf(pct2, sizeof(pct2), "%d%%", (motor2PWM * 100 + 127) / 255);
      snprintf(line, sizeof(line), "M1:%-4s      M2:%-4s", pct1, pct2);
      lcdPrintLine(3, line);
    }

    lastLcdUpdate = millis();
  }
}
