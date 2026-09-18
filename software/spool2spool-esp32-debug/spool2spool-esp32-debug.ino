/*
  Dual motor control, ESP32 (30-pin devkit) — DEBUG COPY, no LCD
  Copy of software/spool2spool-esp32/spool2spool-esp32.ino with every bit of LCD output
  redirected to Serial instead, so the motor/button/angle-serial logic can be tested and
  developed while waiting on a replacement I2C LCD backpack (the original one turned out
  to be dead — see the I2C scan in the main sketch). Once a working LCD is back, keep
  developing on the main sketch, not this one — this file is not meant to be kept in sync
  long-term, just to unblock bench testing right now.

  - Motor 1: closed loop constant speed via encoder/hall pulse feedback, with manual override
    (displayed as line speed in m/min, using the spool diameter set below)
  - Motor 2: speed follows the boom angle, received over Serial2 from the Wemos D1 mini
    angle-sensor node (software/WemosAngleSensor — AS5600 next to the boom pivot)
  - Buttons: LEFT/RIGHT toggle manual/auto mode per motor, UP/DOWN adjust spool diameter,
    SELECT starts the system
  - Both motor outputs stay at zero after power up until SELECT is pressed once
  - Everything that would've gone to the LCD (boot banner, start prompt, diameter changes,
    live speed/angle/mode) now goes to Serial instead

  Wiring (30-pin ESP32 devkit):
  - Serial2 RX -> GPIO16, from the Wemos D1 mini's TX (one-way link — this board's TX/GPIO17
    is unused, the Wemos never listens)
  - motor1PwmPin -> GPIO25 (motor 1 driver RPWM)
  - motor2PwmPin -> GPIO26 (motor 2 driver RPWM)
  - encoderPin    -> GPIO4  (hall sensor / encoder, interrupt capable — any GPIO works on ESP32)
  - pot1Pin -> GPIO34, pot2Pin -> GPIO35 (ADC1-only pins — avoids ADC2/WiFi conflicts)
  - buttons -> GPIO13 (LEFT), GPIO27 (RIGHT), GPIO32 (UP), GPIO33 (DOWN), GPIO14 (SELECT),
    each wired to GND through a pushbutton, using the internal pull-up (no external resistors)
  - No LCD, no I2C wiring needed for this debug copy at all

  Requires arduino-esp32 core 3.x — that's what gives analogWrite() direct PWM support here,
  same call signature as the original AVR code. Older cores need ledcWrite() instead.
*/

// ---- Motor pins ----
const int motor1PwmPin = 25;  // motor 1, closed loop RPM control
const int motor2PwmPin = 26;  // motor 2, angle controlled

// ---- Encoder / hall pulse pin (motor 1 speed feedback) ----
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
// smooths what's shown on the display/serial, it doesn't change motor control response.
const int speedAvgSamples = 5;  // ~5 seconds of smoothing at the once-per-second update rate
float speedAvgBuffer[speedAvgSamples] = {0};
int speedAvgIndex = 0;
int speedAvgCount = 0;  // samples recorded so far, caps at speedAvgSamples

// ---- Manual override potentiometers ----
const int pot1Pin = 34;  // motor 1 manual speed (ADC1-only pin)
const int pot2Pin = 35;  // motor 2 manual speed (ADC1-only pin)

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

// ---- Motor 1 closed loop control ----
float targetRPM = 1200.0;     // set your target speed here
int motor1PWM = 128;          // starting point, self adjusts
const float Kp = 0.5;         // tune on the bench, raise if slow to correct, lower if it hunts

// ---- Motor 1 spool, for converting RPM to line speed for the display ----
// UP/DOWN adjust this live, e.g. to match whatever spool is currently loaded
// NOTE: this does not persist across power cycles, resets to the placeholder below on reset
float spoolDiameterMM = 50.0;  // placeholder, adjust for the real spool on the bench
const float spoolDiameterStepMM = 1.0;
const float spoolDiameterMinMM = 5.0;
const float spoolDiameterMaxMM = 300.0;

// ---- Motor 2 angle mapping range ----
// adjust these to match your boom's real min/max angle in degrees
const float angleMin = 0.0;
const float angleMax = 90.0;
int motor2PWM = 0;

// ---- Boom angle, received from the Wemos D1 mini angle-sensor node over Serial2 ----
// The Wemos only sends a new line when the angle has moved more than 1 degree (see
// software/WemosAngleSensor), not on a fixed schedule — this just holds the last value
// received until the next line comes in.
float boomAngle = 0.0;
String angleLineBuffer = "";

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

// non-blocking: drains whatever's waiting on Serial2 and updates boomAngle once a full
// line has arrived. Never waits for more data, so this can't stall the main loop.
void updateBoomAngleFromSerial() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      if (angleLineBuffer.length() > 0) {
        boomAngle = angleLineBuffer.toFloat();
      }
      angleLineBuffer = "";
    } else if (c != '\r') {
      angleLineBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);  // matches the ESP32's default ROM bootloader baud -- set the Serial
                          // Monitor to 115200 too, or these prints come out as garbage
  delay(500);  // gives the USB serial monitor time to connect before the first prints
  Serial.println("=== spool2spool-esp32-debug booting (LCD output redirected to Serial) ===");
  Serial.println("EMI Twente");

  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX from the Wemos; TX (GPIO17) is unused
  Serial.println("Serial2 (Wemos angle link) initialized");

  pinMode(btnLeftPin, INPUT_PULLUP);
  pinMode(btnRightPin, INPUT_PULLUP);
  pinMode(btnUpPin, INPUT_PULLUP);
  pinMode(btnDownPin, INPUT_PULLUP);
  pinMode(btnSelectPin, INPUT_PULLUP);
  Serial.println("Button pins configured (INPUT_PULLUP) -- note: with no button wired to a "
                  "pin, it just reads HIGH/not-pressed forever, that's expected");

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, FALLING);

  pinMode(motor1PwmPin, OUTPUT);
  pinMode(motor2PwmPin, OUTPUT);
  analogWrite(motor1PwmPin, 0);
  analogWrite(motor2PwmPin, 0);

  lastCalcTime = millis();
  Serial.println("=== setup() complete, entering loop() ===");
}

unsigned long lastWaitingDebug = 0;

void loop() {
  updateBoomAngleFromSerial();  // non-blocking; keeps boomAngle fresh whenever the Wemos sends

  // ---- both outputs stay at zero until SELECT has been pressed once ----
  if (!started) {
    int startBtn = readLCDButton();
    if (startBtn == 4) {  // SELECT
      started = true;
      lastButton = startBtn;
      Serial.println("SELECT pressed -- started");
    } else {
      analogWrite(motor1PwmPin, 0);
      analogWrite(motor2PwmPin, 0);
      if (millis() - lastWaitingDebug > 2000) {
        Serial.println("Press SELECT to start (waiting on GPIO14 pulled LOW) -- if buttons "
                        "aren't wired yet, this is expected and will just sit here");
        lastWaitingDebug = millis();
      }
      return;  // skip mode/angle/motor logic entirely until started
    }
  }

  // ---- read buttons, act on new press only ----
  int btn = readLCDButton();
  if (btn != -1 && btn != lastButton) {
    if (btn == 3) {  // LEFT toggles motor 1 mode
      manualMode1 = !manualMode1;
      Serial.print("Mode1 -> ");
      Serial.println(manualMode1 ? "MANUAL" : "AUTO");
    }
    if (btn == 0) {  // RIGHT toggles motor 2 mode
      manualMode2 = !manualMode2;
      Serial.print("Mode2 -> ");
      Serial.println(manualMode2 ? "MANUAL" : "AUTO");
    }
    if (btn == 1) {  // UP, increase spool diameter
      spoolDiameterMM = min(spoolDiameterMM + spoolDiameterStepMM, spoolDiameterMaxMM);
      Serial.print("Spool diameter -> ");
      Serial.print(spoolDiameterMM, 0);
      Serial.println("mm");
    }
    if (btn == 2) {  // DOWN, decrease spool diameter
      spoolDiameterMM = max(spoolDiameterMM - spoolDiameterStepMM, spoolDiameterMinMM);
      Serial.print("Spool diameter -> ");
      Serial.print(spoolDiameterMM, 0);
      Serial.println("mm");
    }
  }
  lastButton = btn;

  // ---- motor 2: manual pot or angle mapping ----
  if (manualMode2) {
    int potVal2 = analogRead(pot2Pin);
    motor2PWM = map(potVal2, 0, 4095, 255, 0);  // inverted: potVal 0 = full, 4095 = stop. ESP32 ADC is 12-bit (0-4095), not 10-bit like AVR
  } else {
    float clampedAngle = constrain(boomAngle, angleMin, angleMax);
    motor2PWM = map((long)clampedAngle, (long)angleMin, (long)angleMax, 0, 255);
  }
  analogWrite(motor2PwmPin, motor2PWM);

  // ---- motor 1: manual pot, or closed loop RPM correction once per second ----
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

    if (!manualMode1) {
      float error = targetRPM - currentRPM;
      motor1PWM += (int)(Kp * error / 10);
      motor1PWM = constrain(motor1PWM, 0, 255);
      analogWrite(motor1PwmPin, motor1PWM);
    }

    Serial.print("RPM: ");
    Serial.print(currentRPM);
    Serial.print(" | Speed(m/min): ");
    Serial.print(currentLinearSpeed);
    Serial.print(" | Motor1 PWM: ");
    Serial.print(motor1PWM);
    Serial.print(" | Mode1: ");
    Serial.print(manualMode1 ? "MAN" : "AUTO");
    Serial.print(" | Angle: ");
    Serial.print(boomAngle);
    Serial.print(" | Potval1: ");
    Serial.print(analogRead(pot1Pin));
    Serial.print(" | Potval2: ");
    Serial.print(analogRead(pot2Pin));
    Serial.print(" | Mode2: ");
    Serial.print(manualMode2 ? "MAN" : "AUTO");
    Serial.print(" | Moto2 PWM: ");
    Serial.println(motor2PWM);

    lastCalcTime = millis();
  }
}
