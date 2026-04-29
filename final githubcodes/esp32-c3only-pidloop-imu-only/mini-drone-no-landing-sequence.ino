/*
 * Mini Drone Flight Controller — ESP32-S3
 *
 * SENSORS:
 *   ICM-20948  — IMU (attitude, gyro)          I2C
 *   VL53L1X    — Time of Flight (height hold)   I2C
 *   PMW3901    — Optical Flow (velocity hold)   SPI
 *
 * COMMS:
 *   Controller  → ESP-NOW → Mini Drone  (ARM / TAKEOFF / LAND / sticks)
 *   Motherdrone → ESP-NOW → Mini Drone  (AprilTag position)
 *
 * TASK ARCHITECTURE:
 *   Core 1 | Priority 10 | pidTask     ~1kHz  — attitude estimation + PID + motors
 *   Core 0 | Priority  8 | sensorTask  ~500Hz — ICM-20948 + VL53L1X
 *   Core 0 | Priority  5 | flowTask     50Hz  — PMW3901
 *   Core 0 | Priority  1 | debugTask     5Hz  — serial print
 *
 * FLIGHT STATE MACHINE:
 *   DISARMED → [ARM]      → ARMED
 *   ARMED    → [TAKEOFF]  → TAKING_OFF
 *   TAKING_OFF → [height reached] → FLYING
 *   FLYING   → [LAND]     → DISARMED  (no sequence yet)
 *   Any      → [ESTOP / link loss] → FAILSAFE → DISARMED
 *
 * NOTE: ControlPacket changed — update controller code to match.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "ICM_20948.h"
#include <VL53L1X.h>
#include <Bitcraze_PMW3901.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================================================================
// PIN ASSIGNMENTS — adjust to match your board
// ================================================================
#define SDA_PIN      8    // I2C — ICM-20948 + VL53L1X shared bus
#define SCL_PIN      9

#define SPI_SCK_PIN  12   // SPI — PMW3901 optical flow
#define SPI_MISO_PIN 13
#define SPI_MOSI_PIN 11
#define FLOW_CS_PIN  10

#define MOTOR_FL_PIN 1    // Front Left
#define MOTOR_FR_PIN 2    // Front Right
#define MOTOR_BL_PIN 3    // Back Left
#define MOTOR_BR_PIN 4    // Back Right

// ================================================================
// ESC PARAMS
// ================================================================
#define MIN_US                 1000
#define IDLE_US                1150
#define MAX_US                 2000
#define DISARMED_THROTTLE_US   1100

// ================================================================
// ESP-NOW
// ================================================================
#define ESPNOW_CHANNEL  1
#define RX_TIMEOUT_MS   500

// ================================================================
// FLIGHT PARAMS
// ================================================================
#define TAKEOFF_HEIGHT_MM    800.0f   // target hover height in mm
#define BASE_HOVER_THROTTLE  1260     // starting throttle for takeoff ramp
#define TAKEOFF_RAMP_STEP    3        // us added per PID tick during ramp

// ================================================================
// KNOWN SENDER MACs — update if changed
// ================================================================
uint8_t motherdroneMAC[] = {0x44, 0x1D, 0x64, 0xF7, 0x11, 0xC0};
uint8_t controllerMAC[]  = {0x14, 0x33, 0x5C, 0x0E, 0xD2, 0x34};

// ================================================================
// PACKETS
// NOTE: ControlPacket has changed — controller must send this struct
// ================================================================
typedef struct {
    int16_t throttle;      // 1100–1400 (used for height offset)
    int16_t yaw;           // -100 to 100
    int16_t pitch;         // -100 to 100
    int16_t roll;          // -100 to 100
    bool    arm;           // arm / disarm toggle
    bool    takeoff;       // initiate takeoff
    bool    land;          // land (disarm for now, sequence later)
    bool    emergencyStop; // immediate motor cut
} ControlPacket;

typedef struct {
    int tagId;
    int x;
    int y;
    int size;
    int angle;
} TagData;

// ================================================================
// FLIGHT STATE MACHINE
// ================================================================
enum DroneState {
    STATE_DISARMED,
    STATE_ARMED,
    STATE_TAKING_OFF,
    STATE_FLYING,
    STATE_FAILSAFE
};

volatile DroneState droneState = STATE_DISARMED;

// ================================================================
// SHARED DATA — written/read across tasks, protected by mutexes
// ================================================================
SemaphoreHandle_t cmdMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t heightMutex;
SemaphoreHandle_t flowMutex;
SemaphoreHandle_t tagMutex;

// Command data (from controller)
ControlPacket     sharedCmd = {DISARMED_THROTTLE_US, 0, 0, 0, false, false, false, false};
volatile unsigned long lastPacketMs = 0;
volatile bool     gotPacket         = false;

// IMU / attitude (from sensor task → PID task)
volatile float    shGyroRoll  = 0, shGyroPitch  = 0, shGyroYaw = 0;
volatile float    shRollAngle = 0, shPitchAngle = 0;
volatile bool     imuReady    = false;

// Height (from sensor task → PID task)
volatile float    shHeightMm  = 0;
volatile bool     heightValid = false;

// Optical flow (from flow task → PID task)
volatile int16_t  shFlowDx = 0, shFlowDy = 0;

// AprilTag (from ESP-NOW → PID task)
TagData           sharedTag;
volatile bool     tagReceived = false;

// ================================================================
// SENSORS & MOTORS
// ================================================================
ICM_20948_I2C    icm;
VL53L1X          vlx;
Bitcraze_PMW3901 flow(FLOW_CS_PIN);
Servo            motorFL, motorFR, motorBL, motorBR;

// ================================================================
// PID STRUCT
// ================================================================
struct PID {
    float kp, ki, kd;
    float integral;
    float prevError;
    float prevMeasurement;
    float output;
};

// --- Rate loop PIDs ---
PID pidRollRate  = {0.03f, 0.002f, 0.001f, 0, 0, 0, 0};
PID pidPitchRate = {0.03f, 0.002f, 0.001f, 0, 0, 0, 0};
PID pidYawRate   = {0.10f, 0.005f, 0.0f,   0, 0, 0, 0};

// --- Height PID ---
PID pidHeight    = {0.8f,  0.05f,  0.3f,   0, 0, 0, 0};

// --- Optical flow velocity PIDs (target = 0 = no drift) ---
PID pidFlowX     = {0.3f,  0.01f,  0.05f,  0, 0, 0, 0};
PID pidFlowY     = {0.3f,  0.01f,  0.05f,  0, 0, 0, 0};

// --- Angle loop gains ---
const float ANGLE_KP_ROLL  = 1.5f;
const float ANGLE_KP_PITCH = 1.5f;

// ================================================================
// BETAFLIGHT-STYLE SETTINGS
// ================================================================
const float ITERM_RELAX_THRESHOLD = 10.0f;
const float D_SETPOINT_WEIGHT     = 0.0f;
const float TPA_BREAKPOINT        = 1600.0f;
const float TPA_RATE              = 0.20f;
const float ANTI_GRAVITY_GAIN     = 3.0f;
const float ANTI_GRAVITY_THRESHOLD= 15.0f;

// ================================================================
// FILTER COEFFICIENTS
// ================================================================
const float GYRO_LPF_ALPHA    = 0.97f;
const float OUTPUT_LPF_ALPHA  = 0.75f;
const float TARGET_LPF_ALPHA  = 0.50f;
const float MOTOR_LPF_ALPHA   = 0.60f;
const float ATTITUDE_ALPHA    = 0.98f;  // complementary filter

// ================================================================
// LIMITS
// ================================================================
const float MAX_ROLL_DEG           = 10.0f;
const float MAX_PITCH_DEG          = 10.0f;
const float MAX_YAW_RATE_DPS       = 100.0f;
const float MAX_ANGLE_TO_RATE_DPS  = 30.0f;
const float INTEGRAL_LIMIT         = 30.0f;
const float PID_OUT_LIMIT          = 200.0f;
const float HEIGHT_INTEGRAL_LIMIT  = 150.0f;
const float HEIGHT_OUT_LIMIT       = 200.0f;
const float FLOW_OUT_LIMIT         = 6.0f;   // max angle correction from flow (deg)
const float ANGLE_DEADBAND_DEG     = 2.0f;
const int   CMD_DEADBAND           = 8;
const float NOTCH_CENTER_HZ        = 0.0f;   // set if resonance known, 0 = disabled
const float NOTCH_CUTOFF_HZ        = 0.0f;

// ================================================================
// ATTITUDE STATE (owned by PID task, populated from shared IMU)
// ================================================================
float rollAngleTargetFilt  = 0;
float pitchAngleTargetFilt = 0;
float yawRateTargetFilt    = 0;

float motorFLFilt = MIN_US, motorFRFilt = MIN_US;
float motorBLFilt = MIN_US, motorBRFilt = MIN_US;

float prevThrottle      = DISARMED_THROTTLE_US;
float throttleDelta     = 0;
float antiGravityBoost  = 1.0f;
int   rampThrottle      = BASE_HOVER_THROTTLE;
float targetHeightMm    = TAKEOFF_HEIGHT_MM;

// ================================================================
// GYRO CALIBRATION VALUES
// ================================================================
float gyroRollBias   = 0, gyroPitchBias = 0, gyroYawBias  = 0;
float rollAngleOff   = 0, pitchAngleOff = 0;

// ================================================================
// BIQUAD FILTER
// ================================================================
struct BiquadFilter {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
};

BiquadFilter bqRoll, bqPitch, bqYaw;
BiquadFilter bqRoll2, bqPitch2, bqYaw2;

// ================================================================
// NOTCH FILTER
// ================================================================
struct NotchFilter {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
};

NotchFilter notchRoll, notchPitch, notchYaw;

// ================================================================
// TASK HANDLES
// ================================================================
TaskHandle_t pidTaskHandle    = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t flowTaskHandle   = NULL;
TaskHandle_t debugTaskHandle  = NULL;

unsigned long startTime = 0;

// ================================================================
// FILTER FUNCTIONS
// ================================================================
void initBiquadLPF(BiquadFilter &f, float cutHz, float sampleHz) {
    float w0    = 2.0f * M_PI * cutHz / sampleHz;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) * 0.5f / 0.707f;
    float b0    = (1.0f - cosw0) * 0.5f;
    float b1    = 1.0f - cosw0;
    float b2    = (1.0f - cosw0) * 0.5f;
    float a0    = 1.0f + alpha;
    float a1    = -2.0f * cosw0;
    float a2    = 1.0f - alpha;
    f.b0 = b0/a0; f.b1 = b1/a0; f.b2 = b2/a0;
    f.a1 = a1/a0; f.a2 = a2/a0;
    f.x1 = f.x2 = f.y1 = f.y2 = 0.0f;
}

float applyBiquad(BiquadFilter &f, float x) {
    float y = f.b0*x + f.b1*f.x1 + f.b2*f.x2 - f.a1*f.y1 - f.a2*f.y2;
    f.x2=f.x1; f.x1=x; f.y2=f.y1; f.y1=y;
    return y;
}

void initNotch(NotchFilter &f, float cHz, float bwHz, float sHz) {
    if (cHz <= 0 || bwHz <= 0) {
        f.b0=1; f.b1=f.b2=f.a1=f.a2=0;
        f.x1=f.x2=f.y1=f.y2=0; return;
    }
    float w0=2*M_PI*cHz/sHz, bw=2*M_PI*bwHz/sHz, sw=sinf(w0);
    if (fabsf(sw)<1e-6f){ f.b0=1;f.b1=f.b2=f.a1=f.a2=0;f.x1=f.x2=f.y1=f.y2=0;return; }
    float alpha=sw*sinhf((logf(2)/2)*bw*w0/sw), cw=cosf(w0), a0=1+alpha;
    f.b0=1/a0; f.b1=-2*cw/a0; f.b2=1/a0;
    f.a1=-2*cw/a0; f.a2=(1-alpha)/a0;
    f.x1=f.x2=f.y1=f.y2=0;
}

float applyNotch(NotchFilter &f, float x) {
    float y=f.b0*x+f.b1*f.x1+f.b2*f.x2-f.a1*f.y1-f.a2*f.y2;
    f.x2=f.x1; f.x1=x; f.y2=f.y1; f.y1=y;
    return y;
}

// ================================================================
// HELPERS
// ================================================================
float clampF(float v, float lo, float hi) { return v<lo?lo:v>hi?hi:v; }
int   clampI(int v, int lo, int hi)       { return v<lo?lo:v>hi?hi:v; }
float lpf(float prev, float in, float a)  { return a*prev+(1-a)*in; }
int   dbI(int v, int db)                  { return abs(v)<db?0:v; }
float dbF(float v, float db)              { return fabsf(v)<db?0:v; }

void writeAll(int us) {
    motorFL.writeMicroseconds(us); motorFR.writeMicroseconds(us);
    motorBL.writeMicroseconds(us); motorBR.writeMicroseconds(us);
}

void resetPID(PID &p) { p.integral=p.prevError=p.prevMeasurement=p.output=0; }

void resetAllPID() {
    resetPID(pidRollRate); resetPID(pidPitchRate); resetPID(pidYawRate);
    resetPID(pidHeight);   resetPID(pidFlowX);     resetPID(pidFlowY);
}

void resetTargets() {
    rollAngleTargetFilt=pitchAngleTargetFilt=yawRateTargetFilt=0;
}

void resetMotorFilters(int us) {
    motorFLFilt=motorFRFilt=motorBLFilt=motorBRFilt=(float)us;
}

float calcTPA(int thr) {
    if (thr<=(int)TPA_BREAKPOINT) return 1.0f;
    float t=(float)(thr-(int)TPA_BREAKPOINT)/(float)(MAX_US-(int)TPA_BREAKPOINT);
    return 1.0f-TPA_RATE*clampF(t,0,1);
}

// ================================================================
// PID UPDATE
// ================================================================
float updatePID(PID &pid, float sp, float meas, float dt,
                float tpa, float ag,
                float ilim=INTEGRAL_LIMIT, float olim=PID_OUT_LIMIT) {
    float err = sp - meas;
    float P   = pid.kp * tpa * err;

    float relax = 1.0f;
    float sr = fabsf(sp);
    if (sr > ITERM_RELAX_THRESHOLD)
        relax = clampF(1-(sr-ITERM_RELAX_THRESHOLD)/100, 0, 1);

    pid.integral = clampF(pid.integral + err*dt*relax*ag, -ilim, ilim);
    float I = pid.ki * pid.integral;

    float sdt = dt < 1e-6f ? 1e-6f : dt;
    float dM  = (meas - pid.prevMeasurement) / sdt;
    float dE  = (err  - pid.prevError)        / sdt;
    float D   = pid.kd * tpa * (D_SETPOINT_WEIGHT*dE + (1-D_SETPOINT_WEIGHT)*(-dM));

    pid.prevError       = err;
    pid.prevMeasurement = meas;

    float raw   = clampF(P+I+D, -olim, olim);
    pid.output  = lpf(pid.output, raw, OUTPUT_LPF_ALPHA);
    return pid.output;
}

// ================================================================
// IMU AXIS MAPPING
// ================================================================
void readMappedIMU(float &ax, float &ay, float &az,
                   float &gr, float &gp, float &gy) {
    ax = -icm.accY()/1000.0f;
    ay =  icm.accX()/1000.0f;
    az =  icm.accZ()/1000.0f;
    gr =  icm.gyrX();
    gp =  icm.gyrY();
    gy = -icm.gyrZ();
}

// ================================================================
// GYRO CALIBRATION — called once at boot
// ================================================================
void calibrateGyro() {
    Serial.println("Calibrating — keep still for 3 seconds...");
    unsigned long w = millis();
    while (millis()-w < 3000) { if(icm.dataReady()) icm.getAGMT(); delay(5); }

    const int N=1000; float rs=0,ps=0,ys=0,ras=0,pas=0; int n=0;
    while (n<N) {
        if (icm.dataReady()) {
            icm.getAGMT();
            float ax,ay,az,gr,gp,gy;
            readMappedIMU(ax,ay,az,gr,gp,gy);
            rs+=gr; ps+=gp; ys+=gy;
            ras += atan2f(ay,az)*57.29578f;
            pas += atan2f(-ax,sqrtf(ay*ay+az*az))*57.29578f;
            n++; delay(5);
        }
    }
    gyroRollBias=rs/N; gyroPitchBias=ps/N; gyroYawBias=ys/N;
    rollAngleOff=ras/N; pitchAngleOff=pas/N;
    Serial.printf("Gyro bias R:%.3f P:%.3f Y:%.3f | Level R:%.2f P:%.2f\n",
        gyroRollBias,gyroPitchBias,gyroYawBias,rollAngleOff,pitchAngleOff);
}

// ================================================================
// ESP-NOW RECEIVE CALLBACK
// ================================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

    if (memcmp(info->src_addr, controllerMAC, 6) == 0) {
        if (len == sizeof(ControlPacket)) {
            if (xSemaphoreTake(cmdMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                memcpy(&sharedCmd, data, sizeof(ControlPacket));
                lastPacketMs = millis();
                gotPacket    = true;
                xSemaphoreGive(cmdMutex);
            }
        }
    }

    if (memcmp(info->src_addr, motherdroneMAC, 6) == 0) {
        if (len == sizeof(TagData)) {
            if (xSemaphoreTake(tagMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                memcpy(&sharedTag, data, sizeof(TagData));
                tagReceived = true;
                xSemaphoreGive(tagMutex);
            }
        }
    }
}

// ================================================================
// SENSOR TASK — Core 0, Priority 8
// Reads ICM-20948 and VL53L1X, computes attitude, updates shared vars
// ================================================================
void sensorTask(void *pv) {
    (void)pv;
    static float rollAng=0, pitchAng=0;
    static float grFilt=0, gpFilt=0, gyFilt=0;
    static BiquadFilter bqR,bqP,bqY,bqR2,bqP2,bqY2;
    static NotchFilter  nR,nP,nY;
    static bool filtersInit = false;

    if (!filtersInit) {
        initBiquadLPF(bqR,  80, 1000); initBiquadLPF(bqP,  80, 1000);
        initBiquadLPF(bqY,  80, 1000); initBiquadLPF(bqR2, 80, 1000);
        initBiquadLPF(bqP2, 80, 1000); initBiquadLPF(bqY2, 80, 1000);
        initNotch(nR, NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000);
        initNotch(nP, NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000);
        initNotch(nY, NOTCH_CENTER_HZ, NOTCH_CUTOFF_HZ, 1000);
        filtersInit = true;
    }

    unsigned long lastMicros = micros();
    unsigned long lastVlx    = millis();
    TickType_t    lastWake   = xTaskGetTickCount();

    for (;;) {
        // ---- IMU at max data rate ----
        if (icm.dataReady()) {
            icm.getAGMT();

            unsigned long now = micros();
            float dt = (now - lastMicros) / 1e6f;
            lastMicros = now;
            if (dt < 2e-4f || dt > 0.05f) { vTaskDelayUntil(&lastWake, 1); continue; }

            float ax,ay,az,gr,gp,gy;
            readMappedIMU(ax,ay,az,gr,gp,gy);
            gr -= gyroRollBias; gp -= gyroPitchBias; gy -= gyroYawBias;

            grFilt = lpf(grFilt, gr, GYRO_LPF_ALPHA);
            gpFilt = lpf(gpFilt, gp, GYRO_LPF_ALPHA);
            gyFilt = lpf(gyFilt, gy, GYRO_LPF_ALPHA);

            float fgr = applyNotch(nR, applyBiquad(bqR2, applyBiquad(bqR, grFilt)));
            float fgp = applyNotch(nP, applyBiquad(bqP2, applyBiquad(bqP, gpFilt)));
            float fgy = applyNotch(nY, applyBiquad(bqY2, applyBiquad(bqY, gyFilt)));

            float aRoll  = atan2f(ay,az)*57.29578f - rollAngleOff;
            float aPitch = atan2f(-ax,sqrtf(ay*ay+az*az))*57.29578f - pitchAngleOff;
            float aMag   = sqrtf(ax*ax+ay*ay+az*az);

            if (aMag > 0.75f && aMag < 1.25f) {
                rollAng  = ATTITUDE_ALPHA*(rollAng  + fgr*dt) + (1-ATTITUDE_ALPHA)*aRoll;
                pitchAng = ATTITUDE_ALPHA*(pitchAng + fgp*dt) + (1-ATTITUDE_ALPHA)*aPitch;
            } else {
                rollAng  += fgr * dt;
                pitchAng += fgp * dt;
            }

            // Write to shared vars (volatile float writes are atomic on ESP32)
            shGyroRoll  = fgr; shGyroPitch = fgp; shGyroYaw = fgy;
            shRollAngle = rollAng; shPitchAngle = pitchAng;
            imuReady    = true;
        }

        // ---- VL53L1X at 20Hz ----
        if (millis() - lastVlx >= 50) {
            lastVlx = millis();
            uint16_t dist = vlx.read(false);
            if (!vlx.timeoutOccurred() && dist > 20 && dist < 4000) {
                if (xSemaphoreTake(heightMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    shHeightMm  = (float)dist;
                    heightValid = true;
                    xSemaphoreGive(heightMutex);
                }
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

// ================================================================
// FLOW TASK — Core 0, Priority 5
// Reads PMW3901 at 50Hz
// ================================================================
void flowTask(void *pv) {
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        int16_t dx, dy;
        flow.readMotionCount(&dx, &dy);

        if (xSemaphoreTake(flowMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            shFlowDx = dx;
            shFlowDy = dy;
            xSemaphoreGive(flowMutex);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));  // 50Hz
    }
}

// ================================================================
// DEBUG TASK — Core 0, Priority 1
// ================================================================
void debugTask(void *pv) {
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();
    const char* stateNames[] = {"DISARMED","ARMED","TAKING_OFF","FLYING","FAILSAFE"};

    for (;;) {
        float h = 0;
        if (xSemaphoreTake(heightMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            h = shHeightMm; xSemaphoreGive(heightMutex);
        }

        int16_t fdx=0, fdy=0;
        if (xSemaphoreTake(flowMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            fdx=shFlowDx; fdy=shFlowDy; xSemaphoreGive(flowMutex);
        }

        bool gotTag = false;
        TagData localTag = {};
        if (xSemaphoreTake(tagMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gotTag = tagReceived;
            localTag = sharedTag;
            xSemaphoreGive(tagMutex);
        }

        Serial.printf(
            "State:%-10s | R:%6.2f P:%6.2f | H:%.0fmm | Flow:%d,%d | "
            "FL:%4d FR:%4d BL:%4d BR:%4d\n",
            stateNames[(int)droneState],
            (float)shRollAngle, (float)shPitchAngle,
            h, fdx, fdy,
            (int)motorFLFilt, (int)motorFRFilt,
            (int)motorBLFilt, (int)motorBRFilt
        );

        if (gotTag) {
            Serial.printf("  Tag ID:%d  X:%d  Y:%d  Size:%d  Angle:%d\n",
                localTag.tagId, localTag.x, localTag.y, localTag.size, localTag.angle);
        } else {
            Serial.printf("  Tag: ---\n");
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200)); // 5Hz
    }
}

// ================================================================
// PID TASK — Core 1, Priority 10 (highest — never interrupted)
// ================================================================
void pidTask(void *pv) {
    (void)pv;
    unsigned long lastUs = micros();

    for (;;) {
        // ---- Timing ----
        unsigned long now = micros();
        float dt = (now - lastUs) / 1e6f;
        lastUs = now;
        if (dt < 2e-4f || dt > 0.05f) { vTaskDelay(1); continue; }

        // ---- Startup settle (5s) ----
        if ((millis() - startTime) < 5000) {
            writeAll(MIN_US);
            resetAllPID(); resetTargets(); resetMotorFilters(MIN_US);
            vTaskDelay(1); continue;
        }

        // ---- Snapshot shared command ----
        ControlPacket cmd;
        unsigned long lastRx;
        bool          hasPkt;
        if (xSemaphoreTake(cmdMutex, 0) == pdTRUE) {
            memcpy(&cmd, &sharedCmd, sizeof(ControlPacket));
            lastRx = lastPacketMs;
            hasPkt = gotPacket;
            xSemaphoreGive(cmdMutex);
        } else { vTaskDelay(1); continue; }

        bool linkAlive = hasPkt && (millis() - lastRx < RX_TIMEOUT_MS);

        // ---- State transitions ----
        if (!linkAlive || cmd.emergencyStop) {
            droneState = STATE_FAILSAFE;
        }

        switch (droneState) {
            case STATE_DISARMED:
                if (linkAlive && !cmd.emergencyStop && cmd.arm)
                    droneState = STATE_ARMED;
                break;

            case STATE_ARMED:
                if (!cmd.arm) { droneState = STATE_DISARMED; break; }
                if (cmd.takeoff) {
                    droneState   = STATE_TAKING_OFF;
                    rampThrottle = BASE_HOVER_THROTTLE;
                    targetHeightMm = TAKEOFF_HEIGHT_MM;
                    resetAllPID();
                }
                break;

            case STATE_TAKING_OFF: {
                if (!cmd.arm || cmd.emergencyStop) { droneState = STATE_DISARMED; break; }
                if (cmd.land)                       { droneState = STATE_DISARMED; break; }
                float h = 0; bool hv = false;
                if (xSemaphoreTake(heightMutex, 0) == pdTRUE) {
                    h = shHeightMm; hv = heightValid; xSemaphoreGive(heightMutex);
                }
                if (hv && h >= targetHeightMm) droneState = STATE_FLYING;
                break;
            }

            case STATE_FLYING:
                if (!cmd.arm || cmd.emergencyStop) { droneState = STATE_DISARMED; break; }
                if (cmd.land)                       { droneState = STATE_DISARMED; break; }
                break;

            case STATE_FAILSAFE:
                if (linkAlive && !cmd.emergencyStop) droneState = STATE_DISARMED;
                break;
        }

        // ---- Execute state ----
        if (droneState == STATE_DISARMED || droneState == STATE_FAILSAFE) {
            writeAll(MIN_US);
            resetAllPID(); resetTargets(); resetMotorFilters(MIN_US);
            prevThrottle = DISARMED_THROTTLE_US;
            vTaskDelay(1); continue;
        }

        if (droneState == STATE_ARMED) {
            writeAll(IDLE_US);
            resetAllPID(); resetTargets(); resetMotorFilters(IDLE_US);
            vTaskDelay(1); continue;
        }

        // ---- Read shared sensor data ----
        float gyroRoll  = shGyroRoll;
        float gyroPitch = shGyroPitch;
        float gyroYaw   = shGyroYaw;
        float rollAngle = shRollAngle;
        float pitchAngle= shPitchAngle;

        float heightMm = 0; bool hValid = false;
        if (xSemaphoreTake(heightMutex, 0) == pdTRUE) {
            heightMm = shHeightMm; hValid = heightValid; xSemaphoreGive(heightMutex);
        }

        int16_t fdx=0, fdy=0;
        if (xSemaphoreTake(flowMutex, 0) == pdTRUE) {
            fdx=shFlowDx; fdy=shFlowDy; xSemaphoreGive(flowMutex);
        }

        // ---- TAKING_OFF: ramp throttle, basic attitude hold ----
        if (droneState == STATE_TAKING_OFF) {
            int thr;
            if (hValid) {
                float hOut = updatePID(pidHeight, targetHeightMm, heightMm,
                                       dt, 1.0f, 1.0f,
                                       HEIGHT_INTEGRAL_LIMIT, HEIGHT_OUT_LIMIT);
                thr = clampI(BASE_HOVER_THROTTLE + (int)hOut, IDLE_US, MAX_US);
            } else {
                rampThrottle = clampI(rampThrottle + TAKEOFF_RAMP_STEP, IDLE_US, BASE_HOVER_THROTTLE + 80);
                thr = rampThrottle;
            }

            float tpa    = calcTPA(thr);
            float rOut   = updatePID(pidRollRate,  0, gyroRoll,  dt, tpa, 1.0f);
            float pOut   = updatePID(pidPitchRate, 0, gyroPitch, dt, tpa, 1.0f);
            float yOut   = updatePID(pidYawRate,   0, gyroYaw,   dt, 1.0f, 1.0f);

            motorFLFilt = lpf(motorFLFilt, thr + pOut + rOut - yOut, MOTOR_LPF_ALPHA);
            motorFRFilt = lpf(motorFRFilt, thr + pOut - rOut + yOut, MOTOR_LPF_ALPHA);
            motorBLFilt = lpf(motorBLFilt, thr - pOut + rOut + yOut, MOTOR_LPF_ALPHA);
            motorBRFilt = lpf(motorBRFilt, thr - pOut - rOut - yOut, MOTOR_LPF_ALPHA);

            motorFL.writeMicroseconds(clampI((int)motorFLFilt, IDLE_US, MAX_US));
            motorFR.writeMicroseconds(clampI((int)motorFRFilt, IDLE_US, MAX_US));
            motorBL.writeMicroseconds(clampI((int)motorBLFilt, IDLE_US, MAX_US));
            motorBR.writeMicroseconds(clampI((int)motorBRFilt, IDLE_US, MAX_US));
            vTaskDelay(1); continue;
        }

        // ================================================================
        // FLYING — full sensor fusion + PID cascade
        // ================================================================

        // Height PID adjusts base throttle
        int baseThrottle = clampI(cmd.throttle, IDLE_US, MAX_US);
        if (hValid) {
            float hOut = updatePID(pidHeight, targetHeightMm, heightMm,
                                   dt, 1.0f, 1.0f,
                                   HEIGHT_INTEGRAL_LIMIT, HEIGHT_OUT_LIMIT);
            baseThrottle = clampI(baseThrottle + (int)hOut, IDLE_US, MAX_US);
        }

        // TPA + anti-gravity
        float tpa = calcTPA(baseThrottle);
        throttleDelta    = fabsf((float)baseThrottle - prevThrottle);
        prevThrottle     = baseThrottle;
        antiGravityBoost = (throttleDelta > ANTI_GRAVITY_THRESHOLD) ? ANTI_GRAVITY_GAIN : 1.0f;

        // Stick inputs
        int rCmd = dbI(cmd.roll,  CMD_DEADBAND);
        int pCmd = dbI(cmd.pitch, CMD_DEADBAND);
        int yCmd = dbI(cmd.yaw,   CMD_DEADBAND);

        float rollTgt  = (rCmd / 100.0f) * MAX_ROLL_DEG;
        float pitchTgt = (pCmd / 100.0f) * MAX_PITCH_DEG;
        float yawTgt   = (yCmd / 100.0f) * MAX_YAW_RATE_DPS;

        // Optical flow velocity hold — only when sticks centred and height valid
        if (abs(rCmd) < CMD_DEADBAND && abs(pCmd) < CMD_DEADBAND && hValid && heightMm > 100) {
            float flowR = updatePID(pidFlowX, 0, (float)fdx, dt, 1.0f, 1.0f,
                                    FLOW_OUT_LIMIT, FLOW_OUT_LIMIT);
            float flowP = updatePID(pidFlowY, 0, (float)fdy, dt, 1.0f, 1.0f,
                                    FLOW_OUT_LIMIT, FLOW_OUT_LIMIT);
            rollTgt  += clampF(flowR, -FLOW_OUT_LIMIT, FLOW_OUT_LIMIT);
            pitchTgt += clampF(flowP, -FLOW_OUT_LIMIT, FLOW_OUT_LIMIT);
        } else {
            resetPID(pidFlowX); resetPID(pidFlowY);
        }

        rollAngleTargetFilt  = lpf(rollAngleTargetFilt,  rollTgt,  TARGET_LPF_ALPHA);
        pitchAngleTargetFilt = lpf(pitchAngleTargetFilt, pitchTgt, TARGET_LPF_ALPHA);
        yawRateTargetFilt    = lpf(yawRateTargetFilt,    yawTgt,   TARGET_LPF_ALPHA);

        // Outer angle loop → rate targets
        float rollRateTgt = clampF(
            (rollAngleTargetFilt  - dbF(rollAngle,  ANGLE_DEADBAND_DEG)) * ANGLE_KP_ROLL,
            -MAX_ANGLE_TO_RATE_DPS, MAX_ANGLE_TO_RATE_DPS);
        float pitchRateTgt = clampF(
            (pitchAngleTargetFilt - dbF(pitchAngle, ANGLE_DEADBAND_DEG)) * ANGLE_KP_PITCH,
            -MAX_ANGLE_TO_RATE_DPS, MAX_ANGLE_TO_RATE_DPS);

        // Inner rate PIDs
        float rOut = updatePID(pidRollRate,  rollRateTgt,      gyroRoll,  dt, tpa, antiGravityBoost);
        float pOut = updatePID(pidPitchRate, pitchRateTgt,     gyroPitch, dt, tpa, antiGravityBoost);
        float yOut = updatePID(pidYawRate,   yawRateTargetFilt, gyroYaw,  dt, 1.0f, 1.0f);

        // Motor mix
        float mFL = baseThrottle + pOut + rOut - yOut;
        float mFR = baseThrottle + pOut - rOut + yOut;
        float mBL = baseThrottle - pOut + rOut + yOut;
        float mBR = baseThrottle - pOut - rOut - yOut;

        motorFLFilt = lpf(motorFLFilt, mFL, MOTOR_LPF_ALPHA);
        motorFRFilt = lpf(motorFRFilt, mFR, MOTOR_LPF_ALPHA);
        motorBLFilt = lpf(motorBLFilt, mBL, MOTOR_LPF_ALPHA);
        motorBRFilt = lpf(motorBRFilt, mBR, MOTOR_LPF_ALPHA);

        motorFL.writeMicroseconds(clampI((int)roundf(motorFLFilt), IDLE_US, MAX_US));
        motorFR.writeMicroseconds(clampI((int)roundf(motorFRFilt), IDLE_US, MAX_US));
        motorBL.writeMicroseconds(clampI((int)roundf(motorBLFilt), IDLE_US, MAX_US));
        motorBR.writeMicroseconds(clampI((int)roundf(motorBRFilt), IDLE_US, MAX_US));

        vTaskDelay(1);
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Mutexes
    cmdMutex    = xSemaphoreCreateMutex();
    imuMutex    = xSemaphoreCreateMutex();
    heightMutex = xSemaphoreCreateMutex();
    flowMutex   = xSemaphoreCreateMutex();
    tagMutex    = xSemaphoreCreateMutex();

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Motors
    motorFL.attach(MOTOR_FL_PIN, MIN_US, MAX_US);
    motorFR.attach(MOTOR_FR_PIN, MIN_US, MAX_US);
    motorBL.attach(MOTOR_BL_PIN, MIN_US, MAX_US);
    motorBR.attach(MOTOR_BR_PIN, MIN_US, MAX_US);
    writeAll(MIN_US);
    resetMotorFilters(MIN_US);

    // ICM-20948
    uint8_t addr = 0;
    for (int i=0; i<5; i++) {
        if (icm.begin(Wire,0x68)==ICM_20948_Stat_Ok){addr=0x68;break;}
        if (icm.begin(Wire,0x69)==ICM_20948_Stat_Ok){addr=0x69;break;}
        delay(200);
    }
    if (!addr) { Serial.println("ICM-20948 not found"); while(1) delay(100); }
    Serial.printf("ICM found at 0x%02X\n", addr);

    ICM_20948_dlpcfg_t dlpf;
    dlpf.g = gyr_d23bw9_n35bw9;
    dlpf.a = acc_d50bw4_n68bw8;
    icm.setDLPFcfg(ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc, dlpf);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);

    // VL53L1X
    vlx.setTimeout(500);
    if (!vlx.init()) { Serial.println("VL53L1X not found"); while(1) delay(100); }
    vlx.setDistanceMode(VL53L1X::Long);
    vlx.setMeasurementTimingBudget(50000);
    vlx.startContinuous(50);
    Serial.println("VL53L1X OK");

    // PMW3901
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, FLOW_CS_PIN);
    if (!flow.begin()) { Serial.println("PMW3901 not found"); while(1) delay(100); }
    Serial.println("PMW3901 OK");

    // Gyro calibration
    calibrateGyro();

    // WiFi / ESP-NOW
    WiFi.mode(WIFI_STA);
    delay(200);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW failed"); while(1) delay(100); }
    esp_now_register_recv_cb(onDataRecv);

    Serial.print("Mini Drone MAC: ");
    Serial.println(WiFi.macAddress());

    // FreeRTOS tasks
    //                                      name      stack  pv  pri  handle           core
    xTaskCreatePinnedToCore(pidTask,    "PID",    8192, NULL, 10, &pidTaskHandle,    1); // highest
    xTaskCreatePinnedToCore(sensorTask, "SENSOR", 6144, NULL,  8, &sensorTaskHandle, 0);
    xTaskCreatePinnedToCore(flowTask,   "FLOW",   4096, NULL,  5, &flowTaskHandle,   0);
    xTaskCreatePinnedToCore(debugTask,  "DEBUG",  4096, NULL,  1, &debugTaskHandle,  0); // lowest

    startTime = millis();
    Serial.println("Mini Drone ready — waiting for controller");
}

// ================================================================
// LOOP — FreeRTOS handles everything, loop just sleeps
// ================================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}