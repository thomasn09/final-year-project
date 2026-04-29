
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ── Motor pins ────────────────────────────────────────────────────
#define MOTOR_FL_PIN  4
#define MOTOR_FR_PIN  5
#define MOTOR_BL_PIN  3
#define MOTOR_BR_PIN 1
#define UART_RX_PIN  20    // S3 TX → C3 GPIO20

// ── ESC ───────────────────────────────────────────────────────────
#define MIN_US                  1000
#define IDLE_US                 1200
#define MAX_US                  2000
#define PWM_FREQ                  50
#define PWM_RES                   16
#define ARM_THROTTLE_US         1120   // throttle must be below this to arm
#define PID_ENABLE_THROTTLE_US  1100   // PID kicks in above this

// ── ESP-NOW ───────────────────────────────────────────────────────
#define ESPNOW_CHANNEL 1
#define RX_TIMEOUT_MS  500

// ── Shared structs ────────────────────────────────────────────────
#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  header;
  float    roll;
  float    pitch;
  float    gyrX;
  float    gyrY;
  float    gyrZ;
  float    heightMM;
  float    velZ;
  int16_t  flowX;
  int16_t  flowY;
  uint8_t  valid;
  uint8_t  checksum;
};

struct ControlPacket {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  bool    hoverMode;
  bool    emergencyStop;
  bool    arm;
  bool    takeoff;
  bool    land;
};

struct DroneStatusPacket {
  bool     droneSeen;
  float    tagX;
  float    tagY;
  uint16_t tagSize;
  int16_t  tagAngle;
  uint8_t  tagId;
};
#pragma pack(pop)

// ── PID struct ────────────────────────────────────────────────────
struct PID {
  float kp, ki, kd;
  float integral        = 0.0f;
  float prevError       = 0.0f;
  float prevMeasurement = 0.0f;
  float output          = 0.0f;
};

// Rate loop gains
PID pidRollRate  = {0.03f, 0.002f, 0.001f};
PID pidPitchRate = {0.03f, 0.002f, 0.001f};
PID pidYawRate   = {0.10f, 0.005f, 0.000f};

// Angle outer loop
const float ANGLE_KP_ROLL  = 1.5f;
const float ANGLE_KP_PITCH = 1.5f;

// Height PID
PID pidHeight = {0.80f, 0.020f, 0.0f};
const float HEIGHT_D_VELZ   = 0.50f;
const float BASE_THROTTLE   = 1380.0f;
const float TARGET_HEIGHT   = 300.0f;   // mm

// ── Betaflight features ───────────────────────────────────────────
const float ITERM_RELAX_THRESHOLD  = 10.0f;
const float D_SETPOINT_WEIGHT      = 0.0f;
const float TPA_BREAKPOINT         = 1600.0f;
const float TPA_RATE               = 0.20f;
const float ANTI_GRAVITY_GAIN      = 3.0f;
const float ANTI_GRAVITY_THRESHOLD = 15.0f;
const float PID_OUTPUT_LIMIT       = 200.0f;
const float INTEGRAL_LIMIT         = 30.0f;
const float MAX_ROLL_ANGLE_DEG     = 10.0f;
const float MAX_PITCH_ANGLE_DEG    = 10.0f;
const float MAX_YAW_RATE_DPS       = 100.0f;
const float MAX_ANGLE_TO_RATE_DPS  = 30.0f;
const float ANGLE_DEADBAND_DEG     = 2.0f;
const int   CMD_DEADBAND           = 8;

// ── Filter constants ──────────────────────────────────────────────
const float gyroLPF   = 0.97f;
const float outputLPF = 0.75f;
const float targetLPF = 0.50f;
const float motorLPF  = 0.60f;

// ── Optical flow position hold ────────────────────────────────────
const float FLOW_KP             = 0.0f;   // start at 0, tune up (try 0.05)
const float FLOW_X_SIGN         = 1.0f;   // flip to -1 if X correction reversed
const float FLOW_Y_SIGN         = 1.0f;   // flip to -1 if Y correction reversed
const float FLOW_MAX_CORRECTION = 8.0f;   // deg/s cap on flow influence

// ── Biquad filter ─────────────────────────────────────────────────
struct BiquadFilter {
  float b0=0, b1=0, b2=0, a1=0, a2=0;
  float x1=0, x2=0, y1=0, y2=0;

  void initLPF(float cutoffHz, float sampleHz) {
    float w0    = 2.0f * M_PI * cutoffHz / sampleHz;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) * 0.5f * (1.0f / 0.707f);
    float _b0   = (1.0f - cosw0) / 2.0f;
    float _b1   = 1.0f - cosw0;
    float _b2   = (1.0f - cosw0) / 2.0f;
    float a0    = 1.0f + alpha;
    b0 = _b0/a0; b1 = _b1/a0; b2 = _b2/a0;
    a1 = (-2.0f*cosw0)/a0; a2 = (1.0f-alpha)/a0;
    x1 = x2 = y1 = y2 = 0.0f;
  }

  float apply(float x) {
    float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    x2=x1; x1=x; y2=y1; y1=y;
    return y;
  }
};

BiquadFilter bqRoll, bqPitch, bqYaw;
BiquadFilter bqRoll2, bqPitch2, bqYaw2;

// ── Notch filter ──────────────────────────────────────────────────
struct NotchFilter {
  float b0=1, b1=0, b2=0, a1=0, a2=0;
  float x1=0, x2=0, y1=0, y2=0;

  void init(float centerHz, float cutoffHz, float sampleHz) {
    if (centerHz <= 0.0f || cutoffHz <= 0.0f) return;
    float w0    = 2.0f * M_PI * centerHz / sampleHz;
    float sinw0 = sinf(w0);
    if (fabsf(sinw0) < 1e-6f) return;
    float bw    = 2.0f * M_PI * cutoffHz / sampleHz;
    float alpha = sinw0 * sinhf((logf(2.0f)/2.0f) * bw * w0 / sinw0);
    float cosw0 = cosf(w0);
    float a0    = 1.0f + alpha;
    b0 = 1.0f/a0; b1 = -2.0f*cosw0/a0; b2 = 1.0f/a0;
    a1 = -2.0f*cosw0/a0; a2 = (1.0f-alpha)/a0;
  }

  float apply(float x) {
    float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    x2=x1; x1=x; y2=y1; y1=y;
    return y;
  }
};

NotchFilter notchRoll, notchPitch, notchYaw;
const float NOTCH_CENTER_HZ = 0.0f;
const float NOTCH_CUTOFF_HZ = 0.0f;

// ── State machine ─────────────────────────────────────────────────
enum FlightState { DISARMED, ARMED, TAKING_OFF, HOVERING, LANDING };
FlightState state = DISARMED;
const char* stateNames[] = { "DISARMED", "ARMED   ", "TAKEOFF ", "HOVERING", "LANDING " };

float targetHeightMM  = TARGET_HEIGHT;
unsigned long stateStartMs = 0;
float         landStartMM  = 0.0f;
const float   TAKEOFF_RAMP_MS = 2500.0f;
const float   LAND_RATE_MM_S  = 80.0f;
const float   LAND_CUT_MM     = 60.0f;

// ── Attitude state ────────────────────────────────────────────────
float gyroRoll  = 0, gyroPitch = 0, gyroYaw = 0;
float gyroRollFilt = 0, gyroPitchFilt = 0, gyroYawFilt = 0;
float rollAngleTargetFilt = 0, pitchAngleTargetFilt = 0, yawRateTargetFilt = 0;
float gyroRollBias = 0, gyroPitchBias = 0, gyroYawBias = 0;
bool  gyroCalibratedOK = false;

// ── Motor filter state ────────────────────────────────────────────
float motorFLFilt = MIN_US, motorFRFilt = MIN_US;
float motorBLFilt = MIN_US, motorBRFilt = MIN_US;

// ── VelZ extra LPF (C3 side) ──────────────────────────────────────
float velZFilt = 0.0f;
const float VELZ_LPF_ALPHA = 0.80f;
const float VELZ_DEADBAND  = 4.0f;
const int   FLOW_DEADBAND  = 25;

// ── Hover button edge tracking ────────────────────────────────────
bool prevHoverMode = false;

// ── Anti-gravity ──────────────────────────────────────────────────
float prevThrottle     = 1000.0f;
float antiGravityBoost = 1.0f;

// ── Shared data ───────────────────────────────────────────────────
SensorPacket    latestSensor;
bool            sensorFresh        = false;
bool            sensorEverReceived = false;

ControlPacket   latestCtrl;
bool            ctrlFresh        = false;
bool            ctrlEverReceived = false;
unsigned long   lastCtrlMs       = 0;

DroneStatusPacket latestDetection;

// ── Timing ────────────────────────────────────────────────────────
unsigned long lastLoopUs  = 0;
unsigned long lastPrintMs = 0;
unsigned long startMs     = 0;

// ── UART parser ───────────────────────────────────────────────────
uint8_t rxBuf[64];
uint8_t rxIdx = 0;

uint8_t calcChecksum(const uint8_t* d, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 1; i < len-1; i++) cs ^= d[i];
  return cs;
}

void parseUART() {
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (rxIdx == 0 && b != 0xAA) continue;
    rxBuf[rxIdx++] = b;
    if (rxIdx >= sizeof(SensorPacket)) {
      rxIdx = 0;
      SensorPacket* p = (SensorPacket*)rxBuf;
      if (p->checksum == calcChecksum(rxBuf, sizeof(SensorPacket))) {
        memcpy(&latestSensor, rxBuf, sizeof(SensorPacket));
        sensorFresh = sensorEverReceived = true;
      }
    }
  }
}

// ── ESP-NOW callback ──────────────────────────────────────────────
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == (int)sizeof(ControlPacket)) {
    memcpy(&latestCtrl, data, len);
    ctrlFresh        = true;
    ctrlEverReceived = true;
    lastCtrlMs       = millis();
  } else if (len == (int)sizeof(DroneStatusPacket)) {
    memcpy(&latestDetection, data, len);
    static bool prevSeen = false;
    if (latestDetection.droneSeen && !prevSeen) {
      Serial.println("========== PI TAG SEEN ==========");
      Serial.printf("  ID:%d  X:%.2f  Y:%.2f  Size:%d\n",
        latestDetection.tagId, latestDetection.tagX,
        latestDetection.tagY,  latestDetection.tagSize);
      Serial.println("Press LAND on controller");
      Serial.println("=================================");
    } else if (!latestDetection.droneSeen && prevSeen) {
      Serial.println("PI TAG LOST");
    }
    prevSeen = latestDetection.droneSeen;
  }
}

// ── LEDC helpers ──────────────────────────────────────────────────
uint32_t usToDuty(int us) {
  return (uint32_t)(constrain(us, MIN_US, MAX_US) / 20000.0f * 65535.0f);
}
void writeMotor(int pin, int us) { ledcWrite(pin, usToDuty(us)); }
void writeAll(int us) {
  writeMotor(MOTOR_FL_PIN, us); writeMotor(MOTOR_FR_PIN, us);
  writeMotor(MOTOR_BL_PIN, us); writeMotor(MOTOR_BR_PIN, us);
}

// ── Helpers ───────────────────────────────────────────────────────
float lpf(float prev, float in, float alpha) { return alpha*prev + (1.0f-alpha)*in; }
float clampF(float v, float lo, float hi)    { return v<lo?lo:v>hi?hi:v; }
int   applyDBi(int v, int db)                { return abs(v)<db?0:v; }
float applyDBf(float v, float db)            { return fabsf(v)<db?0.0f:v; }

float calcTPA(float thr) {
  if (thr <= TPA_BREAKPOINT) return 1.0f;
  float t = (thr - TPA_BREAKPOINT) / (MAX_US - TPA_BREAKPOINT);
  return 1.0f - TPA_RATE * clampF(t, 0.0f, 1.0f);
}

void resetPID(PID& p) { p.integral=p.prevError=p.prevMeasurement=p.output=0; }
void resetAllPID() {
  resetPID(pidRollRate); resetPID(pidPitchRate);
  resetPID(pidYawRate);  resetPID(pidHeight);
}
void resetTargets() { rollAngleTargetFilt=pitchAngleTargetFilt=yawRateTargetFilt=0; }
void resetMotorFilters(float us = MIN_US) {
  motorFLFilt=motorFRFilt=motorBLFilt=motorBRFilt=us;
}

// ── PID compute ───────────────────────────────────────────────────
float updatePID(PID& pid, float setpoint, float measurement,
                float dt, float tpa, float agBoost) {
  float error = setpoint - measurement;
  float P = pid.kp * tpa * error;

  float relax = 1.0f;
  float sp = fabsf(setpoint);
  if (sp > ITERM_RELAX_THRESHOLD)
    relax = clampF(1.0f - (sp - ITERM_RELAX_THRESHOLD)/100.0f, 0.0f, 1.0f);
  pid.integral = clampF(pid.integral + error*dt*relax*agBoost, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I = pid.ki * pid.integral;

  float safeDt = dt < 1e-6f ? 1e-6f : dt;
  float dMeas  = (measurement - pid.prevMeasurement) / safeDt;
  float dErr   = (error - pid.prevError) / safeDt;
  float dInput = D_SETPOINT_WEIGHT*dErr + (1.0f-D_SETPOINT_WEIGHT)*(-dMeas);
  float D      = pid.kd * tpa * dInput;

  pid.prevError       = error;
  pid.prevMeasurement = measurement;

  float raw = clampF(P+I+D, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  pid.output = lpf(pid.output, raw, outputLPF);
  return pid.output;
}

// ── Height PID ────────────────────────────────────────────────────
float computePIDHeight(float targetMM, float velZ, float dt) {
  float err  = targetMM - latestSensor.heightMM;
  pidHeight.integral = constrain(pidHeight.integral + err * dt, -250.0f, 250.0f);
  float hOut = pidHeight.kp * err
             + pidHeight.ki * pidHeight.integral
             - HEIGHT_D_VELZ * velZ;
  return constrain(hOut, -300.0f, 300.0f);
}

// ── Gyro calibration ──────────────────────────────────────────────
void calibrateGyro() {
  if (!sensorEverReceived) {
    Serial.println("Skipping gyro cal — no S3 data");
    return;
  }
  Serial.println("Gyro cal — keep still for 3 s...");
  unsigned long t = millis();
  while (millis()-t < 3000) { parseUART(); delay(5); }

  Serial.println("Sampling...");
  float rs=0, ps=0, ys=0;
  int n = 0;
  unsigned long deadline = millis() + 5000;
  while (n < 500 && millis() < deadline) {
    parseUART();
    if (sensorFresh) {
      rs += latestSensor.gyrX;
      ps += latestSensor.gyrY;
      ys += latestSensor.gyrZ;
      n++;
      sensorFresh = false;
    }
    delay(5);
  }
  if (n > 10) {
    gyroRollBias  = rs/n;
    gyroPitchBias = ps/n;
    gyroYawBias   = ys/n;
    gyroCalibratedOK = true;
    Serial.printf("Gyro bias  R:%.3f  P:%.3f  Y:%.3f  (n=%d)\n",
      gyroRollBias, gyroPitchBias, gyroYawBias, n);
  } else {
    Serial.println("WARNING: Gyro cal failed — check S3 UART link");
  }
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== C3 FLIGHT CONTROLLER BOOT ===");

  // ── LEDC init — all outputs to MIN immediately ─────────────────
  ledcAttach(MOTOR_FL_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_FR_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_BL_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_BR_PIN, PWM_FREQ, PWM_RES);
  writeAll(MIN_US);
  Serial.println("Motors: FL=GPIO4 FR=GPIO5 BL=GPIO6 BR=GPIO10");

  // ── ESC arm sequence ───────────────────────────────────────────
  // BLHeli ESCs do NOT need MAX/MIN throttle calibration — that
  // sequence was causing unreliable beeps. Just hold MIN while
  // ESCs power up and they will arm automatically.
  //
  // Power-up order:
  //   1. Flash this sketch (USB only)
  //   2. Connect battery — C3 and ESCs power up together
  //   3. C3 immediately outputs MIN; ESCs see MIN within ms and arm
  //   → You should hear consistent beeps on every boot
  Serial.println("\n--- ESC ARM ---");
  Serial.println("Holding MIN (1000us) — connect battery if not already connected");
  Serial.println("Listen for ESC startup beeps...");
  writeAll(MIN_US);
  delay(4000);   // 4 s: enough time for ESCs to power up and arm
  Serial.println("ESC arm window done\n");

  // ── UART to S3 ────────────────────────────────────────────────
  Serial1.begin(500000, SERIAL_8N1, UART_RX_PIN, -1);
  Serial.println("UART on GPIO20 @ 500kbaud");

  // ── ESP-NOW ───────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    Serial.print("ESP-NOW ready  MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println(">>> Paste this MAC into controller.ino as miniDroneMAC[] <<<\n");
  } else {
    Serial.println("ESP-NOW FAILED — check WiFi.mode(WIFI_STA)");
  }

  // ── Wait briefly for S3 data (optional) ───────────────────────
  Serial.println("Waiting up to 3 s for S3 sensor link...");
  unsigned long wait = millis();
  while (!sensorEverReceived && millis()-wait < 3000) { parseUART(); delay(10); }
  if (sensorEverReceived) Serial.println("S3 link OK");
  else                    Serial.println("No S3 data — continuing without sensors (attitude PID disabled)");

  calibrateGyro();

  bqRoll.initLPF(80.0f, 1000.0f);
  bqPitch.initLPF(80.0f, 1000.0f);
  bqYaw.initLPF(80.0f, 1000.0f);
  bqRoll2.initLPF(80.0f, 1000.0f);
  bqPitch2.initLPF(80.0f, 1000.0f);
  bqYaw2.initLPF(80.0f, 1000.0f);
  notchRoll.init(NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000.0f);
  notchPitch.init(NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000.0f);
  notchYaw.init(NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000.0f);

  startMs    = millis();
  lastLoopUs = micros();
  Serial.println("Ready.\n");
  Serial.println("To fly: hold ARM switch, then raise throttle above 1120 us");
  Serial.println("Sensor data is optional for basic motor test\n");
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  if (dt < 0.001f || dt > 0.05f) { lastLoopUs = nowUs; return; }
  lastLoopUs = nowUs;

  parseUART();

  // Settle period after boot
  if (millis() - startMs < 2000) { writeAll(MIN_US); return; }

  // Safe copy of packets
  ControlPacket cmd;
  memcpy(&cmd, &latestCtrl, sizeof(cmd));
  bool linkAlive = ctrlEverReceived && (millis()-lastCtrlMs < RX_TIMEOUT_MS);

  // Sensor snapshot
  float roll   = latestSensor.roll;
  float pitch  = latestSensor.pitch;
  float height = latestSensor.heightMM;
  float velZ   = latestSensor.velZ;
  bool  hgtOK  = sensorEverReceived && (latestSensor.valid & 0x02);

  velZFilt = velZFilt * VELZ_LPF_ALPHA + velZ * (1.0f - VELZ_LPF_ALPHA);
  int throttleCmd = constrain((int)cmd.throttle, MIN_US, MAX_US);

  float tpa            = 1.0f;
  float rollOut        = 0.0f;
  float pitchOut       = 0.0f;
  float yawOut         = 0.0f;
  int   oFL = MIN_US, oFR = MIN_US, oBL = MIN_US, oBR = MIN_US;
  int   thr  = MIN_US;
  bool  runPID = false;

  // ── Failsafe ────────────────────────────────────────────────────
  if (!linkAlive || cmd.emergencyStop) {
    if (state != DISARMED) {
      state = DISARMED;
      Serial.println(">> FAILSAFE — motors off");
    }
    writeAll(MIN_US);
    resetAllPID(); resetTargets(); resetMotorFilters();
    prevThrottle = 1000.0f;
    ctrlFresh = false;

  } else {

    bool hoverPressed  = ctrlFresh && cmd.hoverMode  && !prevHoverMode;
    bool hoverReleased = ctrlFresh && !cmd.hoverMode &&  prevHoverMode;
    prevHoverMode = cmd.hoverMode;

    // ── Arm transition ───────────────────────────────────────────
    // FIX: removed sensorEverReceived requirement so the drone can
    // arm and spin motors even when the S3 is absent (useful for
    // motor testing and for flying without sensor feedback).
    if (cmd.arm && state == DISARMED) {
      state = ARMED;
      Serial.println(">> ARMED");
      if (!sensorEverReceived) Serial.println("   (no S3 sensor — attitude PID bypassed)");
    }

    // ── Disarm ──────────────────────────────────────────────────
    if (!cmd.arm && state != DISARMED) {
      state = DISARMED;
      writeAll(MIN_US);
      resetAllPID(); resetTargets(); resetMotorFilters();
      prevHoverMode = false;
      Serial.println(">> DISARMED");
    }

    // ── Hover button → auto-hover at 30 cm ─────────────────────
    if (hoverPressed && cmd.arm &&
        (state == DISARMED || state == ARMED)) {
      if (!sensorEverReceived) {
        Serial.println("Cannot auto-hover — no S3 sensor link");
      } else {
        state = TAKING_OFF;
        stateStartMs = millis();
        resetAllPID(); resetTargets(); resetMotorFilters(IDLE_US);
        Serial.println(">> HOVER — climbing to 30 cm");
      }
    }

    // ── Hover released → land ───────────────────────────────────
    if (hoverReleased && state == HOVERING) {
      state        = LANDING;
      stateStartMs = millis();
      landStartMM  = height;
      Serial.println(">> HOVER RELEASED — auto-landing");
    }

    // ── Manual TAKEOFF ──────────────────────────────────────────
    if (ctrlFresh && cmd.takeoff && state == ARMED) {
      if (!sensorEverReceived) {
        Serial.println("Cannot takeoff — no S3 sensor link");
      } else {
        state = TAKING_OFF;
        stateStartMs = millis();
        resetAllPID(); resetTargets(); resetMotorFilters(IDLE_US);
        Serial.println(">> TAKING OFF");
      }
    }

    // ── Manual LAND ─────────────────────────────────────────────
    if (ctrlFresh && cmd.land &&
        (state == HOVERING || state == TAKING_OFF || state == ARMED)) {
      state        = LANDING;
      stateStartMs = millis();
      landStartMM  = height;
      Serial.printf(">> LANDING from %.0f mm\n", landStartMM);
    }

    ctrlFresh = false;

    // ── Gyro filter ─────────────────────────────────────────────
    if (sensorEverReceived) {
      float gr = latestSensor.gyrX - gyroRollBias;
      float gp = latestSensor.gyrY - gyroPitchBias;
      float gy = -(latestSensor.gyrZ - gyroYawBias);
      gyroRollFilt  = lpf(gyroRollFilt,  gr, gyroLPF);
      gyroPitchFilt = lpf(gyroPitchFilt, gp, gyroLPF);
      gyroYawFilt   = lpf(gyroYawFilt,   gy, gyroLPF);
      gyroRoll  = notchRoll.apply(bqRoll2.apply(bqRoll.apply(gyroRollFilt)));
      gyroPitch = notchPitch.apply(bqPitch2.apply(bqPitch.apply(gyroPitchFilt)));
      gyroYaw   = notchYaw.apply(bqYaw2.apply(bqYaw.apply(gyroYawFilt)));
    }

    // ── State machine ────────────────────────────────────────────
    if (state == DISARMED) {
      writeAll(MIN_US);
      resetAllPID(); resetTargets(); resetMotorFilters();

    } else if (state == ARMED) {
      if (throttleCmd <= ARM_THROTTLE_US) {
        // Throttle below arm threshold — hold idle
        writeAll(MIN_US);
        resetAllPID(); resetTargets(); resetMotorFilters();
        prevThrottle = 1000.0f;
      } else if (throttleCmd < PID_ENABLE_THROTTLE_US) {
        // Tiny window between arm and PID — pass through directly
        writeAll(throttleCmd);
        resetAllPID(); resetTargets(); resetMotorFilters(throttleCmd);
        prevThrottle = throttleCmd;
      } else {
        // Normal flight — throttle + PID
        thr = throttleCmd;
        if (cmd.hoverMode && hgtOK) {
          float hOut = computePIDHeight(TARGET_HEIGHT, velZFilt, dt);
          thr = (int)(BASE_THROTTLE + hOut);
        }
        runPID = true;
      }

    } else if (state == TAKING_OFF) {
      float frac = constrain(
        (float)(millis()-stateStartMs) / TAKEOFF_RAMP_MS, 0.0f, 1.0f);
      thr = (int)(MIN_US + frac * (BASE_THROTTLE - MIN_US));
      if (frac >= 1.0f) {
        state = HOVERING;
        Serial.println(">> HOVERING");
      }
      runPID = true;

    } else if (state == HOVERING) {
      if (hgtOK) {
        float hOut = computePIDHeight(TARGET_HEIGHT, velZFilt, dt);
        thr = (int)(BASE_THROTTLE + hOut);
      } else {
        thr = (int)BASE_THROTTLE;
      }
      if (!cmd.hoverMode) thr = throttleCmd;
      runPID = true;

    } else if (state == LANDING) {
      float elapsed   = (float)(millis() - stateStartMs);
      float descentMM = LAND_RATE_MM_S * elapsed / 1000.0f;
      float targetLand = landStartMM - descentMM;

      bool forceCut = (elapsed > 10000.0f);
      if ((hgtOK && height < LAND_CUT_MM) || forceCut) {
        writeAll(MIN_US);
        state = DISARMED;
        resetAllPID(); resetMotorFilters();
        Serial.println(forceCut ? ">> LAND TIMEOUT — motors cut" : ">> LANDED");
      } else {
        if (hgtOK) {
          float hOut = computePIDHeight(targetLand, velZFilt, dt);
          thr = (int)(BASE_THROTTLE + constrain(hOut, -200.0f, 100.0f));
        } else {
          thr = constrain(
            (int)(BASE_THROTTLE - elapsed * 0.04f),
            IDLE_US, (int)BASE_THROTTLE);
        }
        runPID = true;
      }
    }

    // ── PID + motor mix ──────────────────────────────────────────
    if (runPID) {
      thr = constrain(thr, IDLE_US, MAX_US);
      tpa = calcTPA((float)thr);

      float throttleDelta = fabsf((float)thr - prevThrottle);
      prevThrottle = (float)thr;
      antiGravityBoost = (throttleDelta > ANTI_GRAVITY_THRESHOLD) ? ANTI_GRAVITY_GAIN : 1.0f;

      float rollRateTarget  = 0.0f;
      float pitchRateTarget = 0.0f;
      float yawRateTarget   = 0.0f;

      if (sensorEverReceived) {
        int rollCmd  = applyDBi((int)cmd.roll,  CMD_DEADBAND);
        int pitchCmd = applyDBi((int)cmd.pitch, CMD_DEADBAND);
        int yawCmd   = applyDBi((int)cmd.yaw,   CMD_DEADBAND);

        float rollTgtRaw  = (rollCmd  / 500.0f) * MAX_ROLL_ANGLE_DEG;
        float pitchTgtRaw = (pitchCmd / 500.0f) * MAX_PITCH_ANGLE_DEG;
        float yawTgtRaw   = (yawCmd   / 500.0f) * MAX_YAW_RATE_DPS;

        rollAngleTargetFilt  = lpf(rollAngleTargetFilt,  rollTgtRaw,  targetLPF);
        pitchAngleTargetFilt = lpf(pitchAngleTargetFilt, pitchTgtRaw, targetLPF);
        yawRateTargetFilt    = lpf(yawRateTargetFilt,    yawTgtRaw,   targetLPF);

        rollRateTarget = clampF(
          (rollAngleTargetFilt  - applyDBf(roll,  ANGLE_DEADBAND_DEG)) * ANGLE_KP_ROLL,
          -MAX_ANGLE_TO_RATE_DPS, MAX_ANGLE_TO_RATE_DPS);
        pitchRateTarget = clampF(
          (pitchAngleTargetFilt - applyDBf(pitch, ANGLE_DEADBAND_DEG)) * ANGLE_KP_PITCH,
          -MAX_ANGLE_TO_RATE_DPS, MAX_ANGLE_TO_RATE_DPS);
        yawRateTarget = yawRateTargetFilt;

        // Optical flow correction (HOVERING only)
        if (state == HOVERING && FLOW_KP > 0.0f && (latestSensor.valid & 0x04)) {
          float hScale = constrain(height / TARGET_HEIGHT, 0.2f, 3.0f);
          rollRateTarget  += clampF(FLOW_X_SIGN*(latestSensor.flowX/10.0f)*FLOW_KP*hScale,
                                    -FLOW_MAX_CORRECTION, FLOW_MAX_CORRECTION);
          pitchRateTarget += clampF(FLOW_Y_SIGN*(latestSensor.flowY/10.0f)*FLOW_KP*hScale,
                                    -FLOW_MAX_CORRECTION, FLOW_MAX_CORRECTION);
        }

        rollOut  = updatePID(pidRollRate,  rollRateTarget,  gyroRoll,  dt, tpa, antiGravityBoost);
        pitchOut = updatePID(pidPitchRate, pitchRateTarget, gyroPitch, dt, tpa, antiGravityBoost);
        yawOut   = updatePID(pidYawRate,   yawRateTarget,   gyroYaw,   dt, 1.0f, 1.0f);
      }
      // If no sensor, PID outputs stay 0 — pure throttle pass-through.
      // Useful for motor spin testing without S3 connected.

      float mFL = thr + pitchOut + rollOut - yawOut;
      float mFR = thr + pitchOut - rollOut + yawOut;
      float mBL = thr - pitchOut + rollOut + yawOut;
      float mBR = thr - pitchOut - rollOut - yawOut;

      motorFLFilt = lpf(motorFLFilt, mFL, motorLPF);
      motorFRFilt = lpf(motorFRFilt, mFR, motorLPF);
      motorBLFilt = lpf(motorBLFilt, mBL, motorLPF);
      motorBRFilt = lpf(motorBRFilt, mBR, motorLPF);

      oFL = constrain((int)roundf(motorFLFilt), IDLE_US, MAX_US);
      oFR = constrain((int)roundf(motorFRFilt), IDLE_US, MAX_US);
      oBL = constrain((int)roundf(motorBLFilt), IDLE_US, MAX_US);
      oBR = constrain((int)roundf(motorBRFilt), IDLE_US, MAX_US);

      writeMotor(MOTOR_FL_PIN, oFL);
      writeMotor(MOTOR_FR_PIN, oFR);
      writeMotor(MOTOR_BL_PIN, oBL);
      writeMotor(MOTOR_BR_PIN, oBR);
    }
  } // end linkAlive block

  // ── Serial debug at 10 Hz ─────────────────────────────────────
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();

    float velZDisp = (fabsf(velZFilt) < VELZ_DEADBAND) ? 0.0f : velZFilt;
    int16_t fxDisp = (abs(latestSensor.flowX) < FLOW_DEADBAND) ? 0 : latestSensor.flowX;
    int16_t fyDisp = (abs(latestSensor.flowY) < FLOW_DEADBAND) ? 0 : latestSensor.flowY;

    if (runPID) {
      Serial.printf(
        "[%s] ARM:%d | Thr:%4d TPA:%.2f | R:%+6.2f P:%+6.2f | gR:%+5.1f gP:%+5.1f gY:%+5.1f\n",
        stateNames[state], (int)cmd.arm,
        thr, tpa, roll, pitch,
        gyroRoll, gyroPitch, gyroYaw);
      Serial.printf(
        "          PID  rOut:%+5.1f pOut:%+5.1f yOut:%+5.1f | H:%5.0fmm Vz:%+5.0f | Fx:%4d Fy:%4d\n",
        rollOut, pitchOut, yawOut, height, velZDisp, fxDisp, fyDisp);
      Serial.printf(
        "          MOTORS  FL:%4d  FR:%4d  BL:%4d  BR:%4d | S3:%s | CTRL:%s\n\n",
        oFL, oFR, oBL, oBR,
        sensorEverReceived ? "OK  " : "NONE",
        linkAlive ? "OK" : "LOST");
    } else {
      Serial.printf(
        "[%s] ARM:%d | Thr:%4d | R:%+6.2f P:%+6.2f | H:%5.0fmm Vz:%+5.0f | S3:%s | CTRL:%s\n\n",
        stateNames[state], (int)cmd.arm,
        throttleCmd, roll, pitch, height, velZDisp,
        sensorEverReceived ? "OK  " : "NONE",
        linkAlive ? "OK" : "LOST");
    }
  }
}