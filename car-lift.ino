
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= PIN CONFIGURATION =================
const byte TRIG_PIN = 8;
const byte ECHO_PIN = 7;

const byte UP_BUTTON   = 6;
const byte DOWN_BUTTON = 5;

const byte BUZZER_PIN = 2;

const byte RED_LED    = 12;
const byte WHITE_LED  = 11;
const byte GREEN_LED  = 10;
const byte YELLOW_LED = 9;

const byte STEP_PIN   = 3;
const byte DIR_PIN    = 4;
const byte ENABLE_PIN = 13;

// Change this to LOW if UP moves in the wrong direction.
const byte UP_DIRECTION = HIGH;

// ================= SENSOR SETTINGS =================
const float MIN_DISTANCE = 5.0;
const float MAX_DISTANCE = 30.0;

const unsigned long SENSOR_INTERVAL_MS = 100;
const unsigned long ECHO_TIMEOUT_US = 12000UL;

// ================= MOTOR SETTINGS =================
// Larger interval = slower motor.
// Start slowly; adjust only after confirming safe driver setup.
const unsigned long STEP_INTERVAL_US = 2000;
const unsigned int STEP_PULSE_US = 5;

// ================= BUTTON SETTINGS =================
const unsigned long DEBOUNCE_MS = 30;
const unsigned long RELEASE_LOCKOUT_MS = 150;

// ================= SENSOR FILTER =================
float samples[5];
byte sampleCount = 0;
byte sampleIndex = 0;

float filteredDistance = -1;
bool sensorError = true;
byte missedEchoes = 0;

unsigned long lastSensorUpdate = 0;

// ================= BUTTON STATE =================
bool upStable = false;
bool downStable = false;

bool upLastRaw = false;
bool downLastRaw = false;

unsigned long upChangeTime = 0;
unsigned long downChangeTime = 0;

bool buttonLockout = false;
unsigned long bothReleasedTime = 0;

// ================= MOTOR STATE =================
enum Motion {
  STOPPED,
  MOVING_UP,
  MOVING_DOWN
};

Motion motion = STOPPED;

unsigned long lastStepTime = 0;

// ================= LCD STATUS =================
String lastLine1 = "";
String lastLine2 = "";

// ================= STARTUP =================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  // A4988 ENABLE is active LOW.
  digitalWrite(ENABLE_PIN, HIGH);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("System Starting");
  lcd.setCursor(0, 1);
  lcd.print("Please Wait...");

  tone(BUZZER_PIN, 880, 100);
  delay(130);
  tone(BUZZER_PIN, 1175, 160);
  delay(180);
  noTone(BUZZER_PIN);

  stopMotor();
  setLEDs(WHITE_LED);

  lcd.clear();
  updateLCD("SYSTEM READY", "Checking sensor");
}

// ================= MAIN LOOP =================
void loop() {
  updateButtons();
  updateSensor();

  // Both buttons pressed: stop and require both released.
  if (upStable && downStable) {
    stopMotor();
    buttonLockout = true;
    bothReleasedTime = 0;

    setLEDs(RED_LED);
    updateLCD("BUTTON CONFLICT", "Release both");
    return;
  }

  if (buttonLockout) {
    stopMotor();
    setLEDs(RED_LED);
    updateLCD("BUTTON CONFLICT", "Release both");

    if (!upStable && !downStable) {
      if (bothReleasedTime == 0) {
        bothReleasedTime = millis();
      }

      if (millis() - bothReleasedTime >= RELEASE_LOCKOUT_MS) {
        buttonLockout = false;
        bothReleasedTime = 0;
        setLEDs(WHITE_LED);
        updateLCD("SYSTEM READY", "Select direction");
      }
    } else {
      bothReleasedTime = 0;
    }
    return;
  }

  // Stop on sensor failure.
  if (sensorError) {
    stopMotor();
    setLEDs(RED_LED);
    updateLCD("SENSOR ERROR", "Check HC-SR04");
    return;
  }

  // Stop if neither button is held.
  if (!upStable && !downStable) {
    stopMotor();
    setLEDs(WHITE_LED);
    updateLCD("SYSTEM READY", distanceText());
    return;
  }

  // UP is blocked if the measured distance exceeds the limit.
  if (upStable) {
    if (filteredDistance > MAX_DISTANCE) {
      stopMotor();
      setLEDs(RED_LED);
      updateLCD("UP BLOCKED", "Distance > 30cm");
      return;
    }

    startMotor(MOVING_UP);
    setLEDs(GREEN_LED);
    updateLCD("MOVING UP", distanceText());
  }

  // DOWN is blocked if the measured distance is below the limit.
  else if (downStable) {
    if (filteredDistance < MIN_DISTANCE) {
      stopMotor();
      setLEDs(RED_LED);
      updateLCD("DOWN BLOCKED", "Distance < 5cm");
      return;
    }

    startMotor(MOVING_DOWN);
    setLEDs(YELLOW_LED);
    updateLCD("MOVING DOWN", distanceText());
  }

  serviceMotor();
}

// ================= BUTTON HANDLING =================
void updateButtons() {
  bool upRaw = (digitalRead(UP_BUTTON) == LOW);
  bool downRaw = (digitalRead(DOWN_BUTTON) == LOW);

  if (upRaw != upLastRaw) {
    upLastRaw = upRaw;
    upChangeTime = millis();
  }

  if (downRaw != downLastRaw) {
    downLastRaw = downRaw;
    downChangeTime = millis();
  }

  if (millis() - upChangeTime >= DEBOUNCE_MS) {
    if (upStable != upRaw) {
      upStable = upRaw;
      if (upStable) buttonBeep();
    }
  }

  if (millis() - downChangeTime >= DEBOUNCE_MS) {
    if (downStable != downRaw) {
      downStable = downRaw;
      if (downStable) buttonBeep();
    }
  }
}

void buttonBeep() {
  tone(BUZZER_PIN, 2500, 65);
}

// ================= ULTRASONIC SENSOR =================
void updateSensor() {
  if (millis() - lastSensorUpdate < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorUpdate = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    if (missedEchoes < 3) missedEchoes++;

    if (missedEchoes >= 3) {
      sensorError = true;
      filteredDistance = -1;
    }
    return;
  }

  float distance = duration * 0.0343 / 2.0;

  if (distance < 2.0 || distance > 200.0) {
    return;
  }

  missedEchoes = 0;
  sensorError = false;

  samples[sampleIndex] = distance;
  sampleIndex = (sampleIndex + 1) % 5;

  if (sampleCount < 5) sampleCount++;

  float median = getMedian();

  if (filteredDistance < 0) {
    filteredDistance = median;
  } else {
    filteredDistance =
        0.30 * median + 0.70 * filteredDistance;
  }
}

float getMedian() {
  float sorted[5];

  for (byte i = 0; i < sampleCount; i++) {
    sorted[i] = samples[i];
  }

  for (byte i = 0; i < sampleCount; i++) {
    for (byte j = i + 1; j < sampleCount; j++) {
      if (sorted[j] < sorted[i]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  return sorted[sampleCount / 2];
}

// ================= MOTOR CONTROL =================
void startMotor(Motion direction) {
  if (motion != direction) {
    digitalWrite(STEP_PIN, LOW);

    if (direction == MOVING_UP) {
      digitalWrite(DIR_PIN, UP_DIRECTION);
    } else {
      digitalWrite(DIR_PIN, !UP_DIRECTION);
    }

    // Allow DIR setup time before stepping.
    delayMicroseconds(10);

    digitalWrite(ENABLE_PIN, LOW);
    motion = direction;
    lastStepTime = micros();
  }
}

void serviceMotor() {
  if (motion == STOPPED) return;

  unsigned long now = micros();

  if (now - lastStepTime >= STEP_INTERVAL_US) {
    lastStepTime = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
  }
}

void stopMotor() {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);
  motion = STOPPED;
}

// ================= LED CONTROL =================
void setLEDs(byte activeLED) {
  digitalWrite(RED_LED,    activeLED == RED_LED);
  digitalWrite(WHITE_LED,  activeLED == WHITE_LED);
  digitalWrite(GREEN_LED,  activeLED == GREEN_LED);
  digitalWrite(YELLOW_LED, activeLED == YELLOW_LED);
}

// ================= LCD HELPERS =================
String distanceText() {
  if (sensorError || filteredDistance < 0) {
    return "Distance: --";
  }

  return "Dist: " + String(filteredDistance, 1) + " cm";
}

void updateLCD(String line1, String line2) {
  if (line1 != lastLine1) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print(line1.substring(0, 16));
    lastLine1 = line1;
  }

  if (line2 != lastLine2) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(line2.substring(0, 16));
    lastLine2 = line2;
  }
}
