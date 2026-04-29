#include <ESP32Servo.h>

Servo esc1, esc2, esc3, esc4;

// Bigger drone ESC pins
const int ESC1_PIN = 17;
const int ESC2_PIN = 16;
const int ESC3_PIN = 4;
const int ESC4_PIN = 2;

// Standard ESC pulse range
const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 2000;

void setup() {
  Serial.begin(115200);

  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);

  Serial.println("ESC calibration starting...");
  Serial.println("Disconnect LiPo now if connected.");
  delay(3000);

  // Send maximum throttle
  esc1.writeMicroseconds(MAX_THROTTLE);
  esc2.writeMicroseconds(MAX_THROTTLE);
  esc3.writeMicroseconds(MAX_THROTTLE);
  esc4.writeMicroseconds(MAX_THROTTLE);

  Serial.println("MAX throttle sent.");
  Serial.println("Now connect LiPo. Wait for calibration beeps.");
  delay(8000);

  // Send minimum throttle
  esc1.writeMicroseconds(MIN_THROTTLE);
  esc2.writeMicroseconds(MIN_THROTTLE);
  esc3.writeMicroseconds(MIN_THROTTLE);
  esc4.writeMicroseconds(MIN_THROTTLE);

  Serial.println("MIN throttle sent.");
  Serial.println("Wait for arming beeps.");
  delay(8000);

  Serial.println("Calibration complete.");
}

void loop() {
  // Keep ESCs armed at minimum throttle
  esc1.writeMicroseconds(MIN_THROTTLE);
  esc2.writeMicroseconds(MIN_THROTTLE);
  esc3.writeMicroseconds(MIN_THROTTLE);
  esc4.writeMicroseconds(MIN_THROTTLE);
}