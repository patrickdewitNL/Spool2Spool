/*
  Dual motor control, Arduino Uno + DFRobot LCD Keypad Shield
  - Motor 1: closed loop constant speed via encoder/hall pulse feedback, with manual override
    (displayed as line speed in m/min, using the spool diameter set below)
  - Motor 2: speed follows boom angle from MPU6050 (I2C), with manual override
  - Shield buttons: LEFT/RIGHT toggle manual/auto mode per motor, UP/DOWN adjust spool diameter,
    SELECT starts the system
  - Both motor outputs stay at zero after power up until SELECT is pressed once
  - LCD shows an "EMI Twente" splash screen at boot, then a "press SELECT to start" prompt, then
    live speed/angle/mode

  Libraries needed:
  - LiquidCrystal.h   (built in)
  - Wire.h            (built in)
  - MPU6050_light.h   (install via Library Manager, search "MPU6050_light" by rfetick)
*/

#include <LiquidCrystal.h>
#include <Wire.h>

// ---- set to 0 if the MPU6050 isn't wired up yet, angle gets simulated instead ----
// ---- flip back to 1 once the sensor is physically present ----
#define USE_MPU6050 0

#if USE_MPU6050
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
#endif

// ---- LCD Keypad Shield pin mapping (standard DFRobot layout) ----
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ---- Motor pins ----
const int motor1PwmPin = 3;   // motor 1, closed loop RPM control
const int motor2PwmPin = 11;  // motor 2, angle controlled

// ---- Encoder / hall pulse pin (motor 1 speed feedback) ----
const int encoderPin = 2;     // hardware interrupt pin
volatile unsigned long pulseCount = 0;
unsigned long lastCalcTime = 0;
float currentRPM = 0;
float currentLinearSpeed = 0;  // m/min, derived from currentRPM and spoolDiameterMM, for display only

// ---- Manual override potentiometers ----
const int pot1Pin = A1;  // motor 1 manual speed
const int pot2Pin = A2;  // motor 2 manual speed

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
// UP/DOWN on the shield adjust this live, e.g. to match whatever spool is currently loaded
// NOTE: this does not persist across power cycles, resets to the placeholder below on reset
float spoolDiameterMM = 50.0;  // placeholder, adjust for the real spool on the bench
const float spoolDiameterStepMM = 1.0;
const float spoolDiameterMinMM = 5.0;
const float spoolDiameterMaxMM = 300.0;
unsigned long diameterDisplayUntil = 0;  // while in the future, LCD shows the diameter instead of normal telemetry
const unsigned long diameterDisplayDuration = 1500;

// ---- Motor 2 angle mapping range ----
// adjust these to match your boom's real min/max angle in degrees
const float angleMin = 0.0;
const float angleMax = 90.0;
int motor2PWM = 0;

// ---- LCD refresh timing ----
unsigned long lastLcdUpdate = 0;

// ---- Shield button reading ----
// standard DFRobot LCD Keypad Shield thresholds on A0
int readLCDButton() {
  int adc = analogRead(A0);
  if (adc > 1000) return -1;  // none
  if (adc < 50)   return 0;   // right
  if (adc < 150)  return 1;   // up
  if (adc < 350)  return 2;   // down
  if (adc < 550)  return 3;   // left
  if (adc < 850)  return 4;   // select
  return -1;
}
int lastButton = -1;

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(3, 0);  // (16 - 10) / 2, centers "EMI Twente" on a 16 column display
  lcd.print("EMI Twente");
  delay(2000);

  Wire.begin();

#if USE_MPU6050
  byte status = mpu.begin();
  while (status != 0) {
    lcd.clear();
    lcd.print("MPU6050 error");
    delay(1000);
    status = mpu.begin();
  }

  lcd.clear();
  lcd.print("Keep boom still");
  delay(1500);
  mpu.calcOffsets();  // boom must be still and level during this step
#else
  lcd.clear();
  lcd.print("DEMO: no MPU");
  delay(1000);
#endif

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, FALLING);

  pinMode(motor1PwmPin, OUTPUT);
  pinMode(motor2PwmPin, OUTPUT);
  analogWrite(motor1PwmPin, 0);
  analogWrite(motor2PwmPin, 0);

  lastCalcTime = millis();
  lcd.clear();
}

void loop() {
  // ---- both outputs stay at zero until SELECT has been pressed once ----
  if (!started) {
    int startBtn = readLCDButton();
    if (startBtn == 4) {  // SELECT
      started = true;
      lastButton = startBtn;
      lcd.clear();
    } else {
      analogWrite(motor1PwmPin, 0);
      analogWrite(motor2PwmPin, 0);
      if (millis() - lastLcdUpdate > 250) {
        lcd.setCursor(0, 0);
        lcd.print("Press SELECT to ");
        lcd.setCursor(0, 1);
        lcd.print("start           ");
        lastLcdUpdate = millis();
      }
      return;  // skip mode/angle/motor logic entirely until started
    }
  }

  // ---- read shield buttons, act on new press only ----
  int btn = readLCDButton();
  if (btn != -1 && btn != lastButton) {
    if (btn == 3) manualMode1 = !manualMode1;  // LEFT toggles motor 1 mode
    if (btn == 0) manualMode2 = !manualMode2;  // RIGHT toggles motor 2 mode

    if (btn == 1) {  // UP, increase spool diameter
      spoolDiameterMM = min(spoolDiameterMM + spoolDiameterStepMM, spoolDiameterMaxMM);
      diameterDisplayUntil = millis() + diameterDisplayDuration;
    }
    if (btn == 2) {  // DOWN, decrease spool diameter
      spoolDiameterMM = max(spoolDiameterMM - spoolDiameterStepMM, spoolDiameterMinMM);
      diameterDisplayUntil = millis() + diameterDisplayDuration;
    }
  }
  lastButton = btn;

#if USE_MPU6050
  mpu.update();
  float boomAngle = mpu.getAngleY();  // check which axis matches your mounting
#else
  // fake angle, slowly sweeps between angleMin and angleMax so you can test
  // motor 2's auto mode logic and the LCD display without real hardware
  float boomAngle = angleMin + (angleMax - angleMin) *
                     (0.5 + 0.5 * sin(millis() / 3000.0));
#endif

  // ---- motor 2: manual pot or angle mapping ----
  if (manualMode2) {
    int potVal2 = analogRead(pot2Pin);
    motor2PWM = map(potVal2, 0, 1023, 0, 255);
  } else {
    float clampedAngle = constrain(boomAngle, angleMin, angleMax);
    motor2PWM = map((long)clampedAngle, (long)angleMin, (long)angleMax, 0, 255);
  }
  analogWrite(motor2PwmPin, motor2PWM);

  // ---- motor 1: manual pot, or closed loop RPM correction once per second ----
  if (manualMode1) {
    int potVal1 = analogRead(pot1Pin);
    motor1PWM = map(potVal1, 0, 1023, 0, 255);
    analogWrite(motor1PwmPin, motor1PWM);
  }

  if (millis() - lastCalcTime >= 1000) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    currentRPM = count * 60.0;  // change multiplier if more than 1 pulse per revolution
    currentLinearSpeed = currentRPM * PI * (spoolDiameterMM / 1000.0);  // m/min

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
    Serial.print(" | Mode2: ");
    Serial.println(manualMode2 ? "MAN" : "AUTO");

    lastCalcTime = millis();
  }

  // ---- LCD update, every 250ms so its readable, not flickering ----
  if (millis() - lastLcdUpdate > 250) {
    if (millis() < diameterDisplayUntil) {
      lcd.setCursor(0, 0);
      lcd.print("Spool diameter  ");
      lcd.setCursor(0, 1);
      lcd.print(spoolDiameterMM, 0);
      lcd.print("mm            ");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Spd:");
      lcd.print(currentLinearSpeed, 1);
      lcd.print("m/min  ");
      lcd.setCursor(13, 0);
      lcd.print("M1");
      lcd.setCursor(15, 0);
      lcd.print(manualMode1 ? "M" : "A");

      lcd.setCursor(0, 1);
      lcd.print("Ang:");
      lcd.print(boomAngle, 1);
      lcd.print("   ");
      lcd.setCursor(13, 1);
      lcd.print("M2");
      lcd.setCursor(15, 1);
      lcd.print(manualMode2 ? "M" : "A");
    }

    lastLcdUpdate = millis();
  }
}
