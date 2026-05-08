#include <ESP32Servo.h>
#include <Wire.h>
#include "ICM_20948.h"

#define MOTOR1_PIN 2   // Front Left
#define MOTOR2_PIN 15  // Front Right
#define MOTOR3_PIN 17  // Back Right
#define MOTOR4_PIN 16  // Back Left

#define MIN_US   1000
#define BASE_US  1150   
#define MAX_US   1300    

Servo motor1, motor2, motor3, motor4;

ICM_20948_I2C imu;

// PID GAINS
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.05;

// STATE
float gyroXFilt = 0, gyroYFilt = 0;
const float LPF_ALPHA = 0.8;   // simple low-pass on gyro

float rollI = 0, pitchI = 0;
float prevRollErr = 0, prevPitchErr = 0;

unsigned long lastTimeUs = 0;

// motor function
void writeAll(int us) {
  motor1.writeMicroseconds(us);
  motor2.writeMicroseconds(us);
  motor3.writeMicroseconds(us);
  motor4.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  delay(1000);


  // Motors
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);
  motor1.attach(MOTOR1_PIN, MIN_US, MAX_US);
  motor2.attach(MOTOR2_PIN, MIN_US, MAX_US);
  motor3.attach(MOTOR3_PIN, MIN_US, MAX_US);
  motor4.attach(MOTOR4_PIN, MIN_US, MAX_US);

  writeAll(MIN_US);
  delay(4000);

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  if (imu.begin(Wire, 0x68) != ICM_20948_Stat_Ok) {
    if (imu.begin(Wire, 0x69) != ICM_20948_Stat_Ok) {
      Serial.println("ICM20948 NOT FOUND");
    }
  }
  Serial.println("IMU ready");

  Serial.println("Starting PID loop");
  lastTimeUs = micros();
}

void loop() {
  // dt 
  unsigned long nowUs = micros();
  float dt = (nowUs - lastTimeUs) / 1e6;
  if (dt > 0.05) dt = 0.05;
  lastTimeUs = nowUs;

  // imu
  if (imu.dataReady()) {
    imu.getAGMT();
    float gx = imu.gyrX();   // roll rate
    float gy = imu.gyrY();   // pitch rate

    // Low-pass filter
    gyroXFilt = LPF_ALPHA * gyroXFilt + (1.0 - LPF_ALPHA) * gx;
    gyroYFilt = LPF_ALPHA * gyroYFilt + (1.0 - LPF_ALPHA) * gy;
  }

  // ----- PID (target = 0 deg/s on both axes) -----
  float rollErr  = 0 - gyroXFilt;
  float pitchErr = 0 - gyroYFilt;

  rollI  += rollErr  * dt;
  pitchI += pitchErr * dt;
  rollI  = constrain(rollI,  -50, 50);
  pitchI = constrain(pitchI, -50, 50);

  float rollD  = (rollErr  - prevRollErr)  / dt;
  float pitchD = (pitchErr - prevPitchErr) / dt;
  prevRollErr  = rollErr;
  prevPitchErr = pitchErr;

  float rollOut  = Kp * rollErr  + Ki * rollI  + Kd * rollD;
  float pitchOut = Kp * pitchErr + Ki * pitchI + Kd * pitchD;

  rollOut  = constrain(rollOut,  -100, 100);
  pitchOut = constrain(pitchOut, -100, 100);

  // ----- Motor mix (X config) -----
  int throttle = BASE_US;
  int m1 = throttle - (int)rollOut - (int)pitchOut;  // FL
  int m2 = throttle + (int)rollOut - (int)pitchOut;  // FR
  int m3 = throttle + (int)rollOut + (int)pitchOut;  // BR
  int m4 = throttle - (int)rollOut + (int)pitchOut;  // BL

  m1 = constrain(m1, MIN_US, MAX_US);
  m2 = constrain(m2, MIN_US, MAX_US);
  m3 = constrain(m3, MIN_US, MAX_US);
  m4 = constrain(m4, MIN_US, MAX_US);

  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);

  // ----- Debug @ 10 Hz -----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf("gX:%+6.1f gY:%+6.1f | rOut:%+5.1f pOut:%+5.1f | M1:%4d M2:%4d M3:%4d M4:%4d\n",
      gyroXFilt, gyroYFilt, rollOut, pitchOut, m1, m2, m3, m4);
  }
}