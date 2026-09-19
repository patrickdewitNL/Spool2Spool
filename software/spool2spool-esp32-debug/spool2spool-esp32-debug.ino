/*
  Dual motor control, ESP32 (30-pin devkit) — DEBUG COPY, no LCD
  Copy of software/spool2spool-esp32/spool2spool-esp32.ino with every bit of LCD output
  redirected to Serial instead, so the motor/button/angle-serial logic can be tested and
  developed while waiting on a replacement I2C LCD backpack (the original one turned out
  to be dead — see the I2C scan in the main sketch). Once a working LCD is back, keep
  developing on the main sketch, not this one — this file is not meant to be kept in sync
  long-term, just to unblock bench testing right now.

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
// smooths what's shown on the display/serial, it doesn't change motor control response.
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
const float Kp = 3.0;                   // raised from 0.5 -- that crawled toward setpoint over
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

// ---- Arm angle, received from the Wemos D1 mini angle-sensor node over Serial2 ----
// The Wemos only sends a new line when the angle has moved more than 1 degree (see
// software/WemosAngleSensor), not on a fixed schedule — this just holds the last value
// received until the next line comes in.
float armAngle = 0.0;
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

  // seed the auto-mode PID starting points from wherever the pots actually are right now,
  // instead of a hardcoded guess -- same inverted mapping as manual mode uses
  motor1PWM = map(analogRead(pot1Pin), 0, 4095, 255, 0);
  motor2PWM = map(analogRead(pot2Pin), 0, 4095, 255, 0);

  lastCalcTime = millis();
  Serial.println("=== setup() complete, entering loop() ===");
}

unsigned long lastWaitingDebug = 0;

void loop() {
  updateArmAngleFromSerial();  // non-blocking; keeps armAngle fresh whenever the Wemos sends

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
    if (btn == 4) {  // SELECT (after start): cycle which value UP/DOWN adjusts
      adjustTarget = (adjustTarget + 1) % 2;
      const char *names[] = {"spool diameter", "motor1 target angle"};
      Serial.print("UP/DOWN now adjusts: ");
      Serial.println(names[adjustTarget]);
    }

    if (btn == 1) {  // UP
      if (adjustTarget == 0) {
        spoolDiameterMM = min(spoolDiameterMM + spoolDiameterStepMM, spoolDiameterMaxMM);
        Serial.print("Spool diameter -> ");
        Serial.print(spoolDiameterMM, 0);
        Serial.println("mm");
      } else {
        targetAngleDeg = min(targetAngleDeg + angleStepDeg, angleMax);
        Serial.print("Motor1 target angle -> ");
        Serial.print(targetAngleDeg, 1);
        Serial.println("deg");
      }
    }
    if (btn == 2) {  // DOWN
      if (adjustTarget == 0) {
        spoolDiameterMM = max(spoolDiameterMM - spoolDiameterStepMM, spoolDiameterMinMM);
        Serial.print("Spool diameter -> ");
        Serial.print(spoolDiameterMM, 0);
        Serial.println("mm");
      } else {
        targetAngleDeg = max(targetAngleDeg - angleStepDeg, angleMin);
        Serial.print("Motor1 target angle -> ");
        Serial.print(targetAngleDeg, 1);
        Serial.println("deg");
      }
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
}
