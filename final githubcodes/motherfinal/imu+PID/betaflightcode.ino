
#include <Wire.h>
#include "ICM_20948.h"
#include <ESP32Servo.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
 
// ================================================================
//  PINS
// ================================================================
#define SDA_PIN      21
#define SCL_PIN      22
#define MOTOR_FL_PIN 15
#define MOTOR_FR_PIN  2
#define MOTOR_BR_PIN 17
#define MOTOR_BL_PIN 16
 
// ================================================================
//  ESC
// ================================================================
#define CAL_MAX_US   2000   // calibration high point (absolute)
#define CAL_MIN_US   1000   // calibration low point (absolute)
#define MIN_US       1020   // flight minimum
#define IDLE_US      1045  // armed idle
#define MAX_US       2000   // flight maximum
 
#define DISARMED_US            1000
#define ARM_THRESHOLD_US       1120
#define PID_ENABLE_THROTTLE_US 1150
 
// Calibration timing
#define CAL_MAX_HOLD_MS  8000   // hold MAX for this long — connect LiPo during this window
#define CAL_MIN_HOLD_MS  8000   // hold MIN for this long — ESCs save range and arm
 
// ================================================================
//  ESP-NOW
// ================================================================
#define ESPNOW_CHANNEL 1
#define RX_TIMEOUT_MS  500
 
typedef struct {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    bool    hoverMode;
    bool    emergencyStop;
} ControlPacket;
 
volatile ControlPacket rxCmd     = {DISARMED_US, 0, 0, 0, false, false};
volatile bool          newPacket = false;
volatile unsigned long lastRxMs  = 0;
volatile unsigned long pktCount  = 0;
 
// ================================================================
//  OBJECTS
// ================================================================
ICM_20948_I2C icm;
Servo motorFL, motorFR, motorBR, motorBL;
 
// ================================================================
//  ATTITUDE
// ================================================================
float rollAngle  = 0.0f, pitchAngle = 0.0f;
float gyroRoll   = 0.0f, gyroPitch  = 0.0f, gyroYaw = 0.0f;
float gyroRollBias = 0.0f, gyroPitchBias = 0.0f, gyroYawBias = 0.0f;
float rollAngleOffset = 0.0f, pitchAngleOffset = 0.0f;
float gyroRollFilt = 0.0f, gyroPitchFilt = 0.0f, gyroYawFilt = 0.0f;
 
// ================================================================
//  MOTOR FILTERS
// ================================================================
float mFLf = MIN_US, mFRf = MIN_US, mBRf = MIN_US, mBLf = MIN_US;
 
// ================================================================
//  SMOOTHED TARGETS
// ================================================================
float rollTargetFilt = 0.0f, pitchTargetFilt = 0.0f, yawRateFilt = 0.0f;
 
// ================================================================
//  TIMING
// ================================================================
unsigned long lastMicros = 0, startTime = 0, lastPrint = 0;
 
// ================================================================
//  PID
// ================================================================
struct PID {
    float kp, ki, kd;
    float integral, prevError, prevMeas, output;
};
 
PID pidRoll  = {0.045f, 0.0030f, 0.0030f, 0,0,0,0};
PID pidPitch = {0.048f, 0.0032f, 0.0035f, 0,0,0,0};
PID pidYaw   = {0.130f, 0.0060f, 0.0000f, 0,0,0,0};
 
const float ANGLE_KP_ROLL  = 7.0f;
const float ANGLE_KP_PITCH = 7.0f;
 
// ================================================================
//  LIMITS
// ================================================================
const float MAX_ROLL_DEG        = 25.0f;
const float MAX_PITCH_DEG       = 25.0f;
const float MAX_YAW_DPS         = 150.0f;
const float MAX_RATE_FROM_ANGLE = 300.0f;
const float INTEGRAL_LIMIT      = 30.0f;
const float PID_OUT_LIMIT       = 400.0f;
const float ANGLE_DEADBAND      = 0.5f;
const int   CMD_DEADBAND        = 5;
 
// ================================================================
//  BETAFLIGHT FEATURES
// ================================================================
const float ITERM_RELAX_THRESH = 10.0f;
const float D_SP_WEIGHT        = 0.0f;
const float TPA_BREAKPOINT     = 1350.0f;
const float TPA_RATE           = 0.20f;
const float ANTI_GRAV_GAIN     = 4.0f;
const float ANTI_GRAV_THRESH   = 15.0f;
const float ANGLE_ALPHA        = 0.98f;
 
// ================================================================
//  FILTER ALPHAS
// ================================================================
const float GYRO_LPF   = 0.80f;
const float OUTPUT_LPF = 0.70f;
const float TARGET_LPF = 0.20f;
const float MOTOR_LPF  = 0.30f;
 
// ================================================================
//  FRAME GEOMETRY (17in roll / 13in pitch)
// ================================================================
const float MIX_ROLL  = 1.000f;
const float MIX_PITCH = 8.5f / 6.5f;
const float MIX_YAW   = 1.000f;
 
float prevThrottle = DISARMED_US;
float agBoost      = 1.0f;
 
// ================================================================
//  BIQUAD
// ================================================================
struct BQ { float b0,b1,b2,a1,a2,x1,x2,y1,y2; };
BQ bqR, bqP, bqY, bqR2, bqP2, bqY2;
 
void initBQ(BQ &f, float hz, float fs) {
    float w0=2.0f*M_PI*hz/fs, cosw=cosf(w0);
    float alpha=sinf(w0)*0.5f*(1.0f/0.707f);
    float b0=(1-cosw)/2, b1=1-cosw, b2=(1-cosw)/2;
    float a0=1+alpha, a1=-2*cosw, a2=1-alpha;
    f.b0=b0/a0; f.b1=b1/a0; f.b2=b2/a0;
    f.a1=a1/a0; f.a2=a2/a0;
    f.x1=f.x2=f.y1=f.y2=0;
}
 
float applyBQ(BQ &f, float x) {
    float y=f.b0*x+f.b1*f.x1+f.b2*f.x2-f.a1*f.y1-f.a2*f.y2;
    f.x2=f.x1; f.x1=x; f.y2=f.y1; f.y1=y; return y;
}
 
// ================================================================
//  HELPERS
// ================================================================
float clampf(float v, float lo, float hi) { return v<lo?lo:v>hi?hi:v; }
float lpf(float p, float i, float a)      { return a*p+(1-a)*i; }
int   dbI(int v, int db)                  { return abs(v)<db?0:v; }
float dbF(float v, float db)              { return fabsf(v)<db?0:v; }
 
void writeAll(int us) {
    motorFL.writeMicroseconds(us); motorFR.writeMicroseconds(us);
    motorBR.writeMicroseconds(us); motorBL.writeMicroseconds(us);
}
 
int armed(int v) { return constrain(v, IDLE_US, MAX_US); }
 
void resetPID(PID &p)  { p.integral=p.prevError=p.prevMeas=p.output=0; }
void resetAllPID()     { resetPID(pidRoll); resetPID(pidPitch); resetPID(pidYaw); }
void resetTargets()    { rollTargetFilt=pitchTargetFilt=yawRateFilt=0; }
void resetMotors(int us) { mFLf=mFRf=mBRf=mBLf=us; }
 
float calcTPA(int thr) {
    if (thr<=(int)TPA_BREAKPOINT) return 1.0f;
    float t=(float)(thr-TPA_BREAKPOINT)/(float)(MAX_US-TPA_BREAKPOINT);
    return 1.0f-TPA_RATE*clampf(t,0,1);
}
 
void readIMU(float &ax, float &ay, float &az, float &gr, float &gp, float &gy) {
    ax= icm.accX()/1000.0f; ay= icm.accY()/1000.0f; az= icm.accZ()/1000.0f;
    gr= icm.gyrX(); gp= icm.gyrY(); gy=-icm.gyrZ();
}
 
float updatePID(PID &pid, float sp, float meas, float dt, float tpa, float ag) {
    float err=sp-meas;
    float P=pid.kp*tpa*err;
    float relax=1.0f;
    if (fabsf(sp)>ITERM_RELAX_THRESH)
        relax=clampf(1-(fabsf(sp)-ITERM_RELAX_THRESH)/100.0f,0,1);
    pid.integral+=err*dt*relax*ag;
    pid.integral=clampf(pid.integral,-INTEGRAL_LIMIT,INTEGRAL_LIMIT);
    float I=pid.ki*pid.integral;
    float safeDt=dt<1e-6f?1e-6f:dt;
    float dM=(meas-pid.prevMeas)/safeDt;
    float dE=(err-pid.prevError)/safeDt;
    float D=pid.kd*tpa*(D_SP_WEIGHT*dE+(1-D_SP_WEIGHT)*(-dM));
    pid.prevError=err; pid.prevMeas=meas;
    float raw=clampf(P+I+D,-PID_OUT_LIMIT,PID_OUT_LIMIT);
    pid.output=lpf(pid.output,raw,OUTPUT_LPF);
    return pid.output;
}
 
// ================================================================
//  ESP-NOW CALLBACK
// ================================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len!=(int)sizeof(ControlPacket)) {
        Serial.printf("BAD PKT len=%d expected=%d\n",len,(int)sizeof(ControlPacket));
        return;
    }
    memcpy((void*)&rxCmd, data, sizeof(ControlPacket));
    newPacket=true;
    lastRxMs=millis();
    pktCount++;
}
 
// ================================================================
//  ESC CALIBRATION — runs every boot
// ================================================================
void calibrateESC() {
    Serial.println("========================================");
    Serial.println(" ESC CALIBRATION");
    Serial.println("========================================");
    Serial.println(" !! PROPS OFF !!");
    Serial.println();
    Serial.println("Sending MAX signal (2000us)...");
    Serial.printf(">>> CONNECT LIPO NOW — you have %d seconds\n", CAL_MAX_HOLD_MS/1000);
 
    writeAll(CAL_MAX_US);
 
    // Count down so you know how long you have
    for (int s = CAL_MAX_HOLD_MS/1000; s > 0; s--) {
        Serial.printf("    %d seconds remaining...\n", s);
        delay(1000);
    }
 
    Serial.println();
    Serial.println("Sending MIN signal (1000us)...");
    Serial.println("Listen for calibration beeps / arming melody...");
 
    writeAll(CAL_MIN_US);
 
    for (int s = CAL_MIN_HOLD_MS/1000; s > 0; s--) {
        Serial.printf("    %d seconds remaining...\n", s);
        delay(1000);
    }
 
    Serial.println();
    Serial.println("ESC calibration complete.");
    Serial.println("========================================");
}
 
// ================================================================
//  GYRO CALIBRATION
// ================================================================
void calibrateGyro() {
    Serial.println("IMU warmup 3s — keep still...");
    unsigned long t=millis();
    while (millis()-t<3000) { if (icm.dataReady()) icm.getAGMT(); delay(5); }
 
    Serial.println("Sampling gyro bias...");
    const int N=1000; float rs=0,ps=0,ys=0,ras=0,pas=0; int n=0;
    while (n<N) {
        if (icm.dataReady()) {
            icm.getAGMT();
            float ax,ay,az,gr,gp,gy;
            readIMU(ax,ay,az,gr,gp,gy);
            rs+=gr; ps+=gp; ys+=gy;
            ras+=atan2f(ay,az)*57.29578f;
            pas+=atan2f(-ax,sqrtf(ay*ay+az*az))*57.29578f;
            n++; delay(5);
        }
    }
    gyroRollBias=rs/N; gyroPitchBias=ps/N; gyroYawBias=ys/N;
    rollAngleOffset=ras/N; pitchAngleOffset=pas/N;
    rollAngle=pitchAngle=0;
    Serial.printf("Gyro bias  R:%.3f P:%.3f Y:%.3f\n",
        gyroRollBias, gyroPitchBias, gyroYawBias);
    Serial.printf("Level off  R:%.3f P:%.3f\n",
        rollAngleOffset, pitchAngleOffset);
}
 
// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== MOTHERDRONE BOOT ===");
    Serial.printf("Packet size: %d bytes\n", (int)sizeof(ControlPacket));
 
    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
 
    // Attach motors with setPeriodHertz(50) — same as working calibration sketch
    motorFL.setPeriodHertz(50); motorFL.attach(MOTOR_FL_PIN, CAL_MIN_US, CAL_MAX_US);
    motorFR.setPeriodHertz(50); motorFR.attach(MOTOR_FR_PIN, CAL_MIN_US, CAL_MAX_US);
    motorBR.setPeriodHertz(50); motorBR.attach(MOTOR_BR_PIN, CAL_MIN_US, CAL_MAX_US);
    motorBL.setPeriodHertz(50); motorBL.attach(MOTOR_BL_PIN, CAL_MIN_US, CAL_MAX_US);
 
    // Safe start
    writeAll(CAL_MIN_US);
    delay(500);
 
    // ---- ESC CALIBRATION ----
    calibrateESC();
 
    // After calibration, settle at MIN before doing anything else
    writeAll(CAL_MIN_US);
    delay(500);
    resetMotors(MIN_US);
 
    // ---- IMU INIT ----
    uint8_t addr=0;
    for (int i=0;i<5;i++) {
        if (icm.begin(Wire,0x68)==ICM_20948_Stat_Ok){addr=0x68;break;}
        if (icm.begin(Wire,0x69)==ICM_20948_Stat_Ok){addr=0x69;break;}
        delay(200);
    }
    if (!addr) { Serial.println("ICM NOT FOUND — halting"); while(1) delay(100); }
    Serial.printf("ICM found at 0x%02X\n", addr);
 
    ICM_20948_dlpcfg_t dlp;
    dlp.g=gyr_d23bw9_n35bw9; dlp.a=acc_d50bw4_n68bw8;
    icm.setDLPFcfg(ICM_20948_Internal_Gyr|ICM_20948_Internal_Acc, dlp);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);
 
    // ---- GYRO CALIBRATION ----
    calibrateGyro();
 
    // ---- FILTERS ----
    initBQ(bqR,80,1000);  initBQ(bqR2,80,1000);
    initBQ(bqP,80,1000);  initBQ(bqP2,80,1000);
    initBQ(bqY,80,1000);  initBQ(bqY2,80,1000);
 
    // ---- ESP-NOW ----
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init()!=ESP_OK) { Serial.println("ESP-NOW FAILED"); while(1) delay(100); }
    esp_now_register_recv_cb(onDataRecv);
 
    Serial.printf("Drone MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("Boot complete — waiting for controller\n");
 
    startTime=millis();
    lastMicros=micros();
}
 
// ================================================================
//  LOOP
// ================================================================
void loop() {
    unsigned long nowUs=micros();
    float dt=(nowUs-lastMicros)/1e6f;
    lastMicros=nowUs;
 
    // ---- Safe RX copy ----
    ControlPacket cmd;
    unsigned long rxAge;
    bool got;
    noInterrupts();
    memcpy(&cmd,(const void*)&rxCmd, sizeof(ControlPacket));
    rxAge=millis()-lastRxMs;
    got=newPacket;
    interrupts();
 
    bool live     = got && (rxAge < RX_TIMEOUT_MS);
    bool settling = (millis()-startTime < 2000);
 
    // ---- Debug ----
    if (millis()-lastPrint > 250) {
        lastPrint=millis();
        const char* state;
        if      (settling)                              state="SETTLING ";
        else if (!live)                                 state="NO_LINK  ";
        else if (cmd.emergencyStop)                     state="E-STOP   ";
        else if (cmd.throttle<=ARM_THRESHOLD_US)        state="DISARMED ";
        else if (cmd.throttle<PID_ENABLE_THROTTLE_US)   state="ARMED_LOW";
        else                                            state="FLYING   ";
 
        Serial.printf("[%s] pkts=%lu rxAge=%lums thr=%d R=%d P=%d Y=%d | ang R:%.1f P:%.1f | FL:%d FR:%d BR:%d BL:%d\n",
            state, pktCount, rxAge,
            cmd.throttle, cmd.roll, cmd.pitch, cmd.yaw,
            rollAngle, pitchAngle,
            (int)mFLf,(int)mFRf,(int)mBRf,(int)mBLf);
    }
 
    if (dt>0.05f) return;
 
    // ---- IMU ----
    if (icm.dataReady()) {
        icm.getAGMT();
        float ax,ay,az,gr,gp,gy;
        readIMU(ax,ay,az,gr,gp,gy);
        gr-=gyroRollBias; gp-=gyroPitchBias; gy-=gyroYawBias;
        gyroRollFilt =lpf(gyroRollFilt, gr, GYRO_LPF);
        gyroPitchFilt=lpf(gyroPitchFilt,gp, GYRO_LPF);
        gyroYawFilt  =lpf(gyroYawFilt,  gy, GYRO_LPF);
        gyroRoll =applyBQ(bqR2,applyBQ(bqR, gyroRollFilt));
        gyroPitch=applyBQ(bqP2,applyBQ(bqP, gyroPitchFilt));
        gyroYaw  =applyBQ(bqY2,applyBQ(bqY, gyroYawFilt));
        float aRoll =atan2f(ay,az)*57.29578f-rollAngleOffset;
        float aPitch=atan2f(-ax,sqrtf(ay*ay+az*az))*57.29578f-pitchAngleOffset;
        float mag=sqrtf(ax*ax+ay*ay+az*az);
        if (mag>0.75f && mag<1.25f) {
            rollAngle =ANGLE_ALPHA*(rollAngle +gyroRoll *dt)+(1-ANGLE_ALPHA)*aRoll;
            pitchAngle=ANGLE_ALPHA*(pitchAngle+gyroPitch*dt)+(1-ANGLE_ALPHA)*aPitch;
        } else {
            rollAngle+=gyroRoll*dt; pitchAngle+=gyroPitch*dt;
        }
    }
 
    // ---- Startup settle ----
    if (settling) {
        writeAll(MIN_US);
        resetAllPID(); resetTargets(); resetMotors(MIN_US);
        return;
    }
 
    // ---- Failsafe ----
    if (!live || cmd.emergencyStop) {
        writeAll(MIN_US);
        resetAllPID(); resetTargets(); resetMotors(MIN_US);
        prevThrottle=DISARMED_US;
        return;
    }
 
    int thr=constrain((int)cmd.throttle, MIN_US, MAX_US);
 
    // ---- Disarmed ----
    if (thr<=ARM_THRESHOLD_US) {
        writeAll(MIN_US);
        resetAllPID(); resetTargets(); resetMotors(MIN_US);
        prevThrottle=DISARMED_US;
        return;
    }
 
    // ---- Armed, below PID threshold ----
    if (thr<PID_ENABLE_THROTTLE_US) {
        writeAll(thr);
        resetAllPID(); resetTargets(); resetMotors(thr);
        prevThrottle=thr;
        return;
    }
 
    // ---- Full PID flight ----
    float tpa=calcTPA(thr);
    float delta=fabsf((float)thr-prevThrottle);
    prevThrottle=thr;
    agBoost=(delta>ANTI_GRAV_THRESH)?ANTI_GRAV_GAIN:1.0f;
 
    float rollTgt =(dbI(cmd.roll, CMD_DEADBAND)/100.0f)*MAX_ROLL_DEG;
    float pitchTgt=(dbI(cmd.pitch,CMD_DEADBAND)/100.0f)*MAX_PITCH_DEG;
    float yawTgt  =(dbI(cmd.yaw,  CMD_DEADBAND)/100.0f)*MAX_YAW_DPS;
 
    rollTargetFilt =lpf(rollTargetFilt, rollTgt, TARGET_LPF);
    pitchTargetFilt=lpf(pitchTargetFilt,pitchTgt,TARGET_LPF);
    yawRateFilt    =lpf(yawRateFilt,    yawTgt,  TARGET_LPF);
 
    float rollRateSP =clampf((rollTargetFilt -dbF(rollAngle, ANGLE_DEADBAND))*ANGLE_KP_ROLL,
                             -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);
    float pitchRateSP=clampf((pitchTargetFilt-dbF(pitchAngle,ANGLE_DEADBAND))*ANGLE_KP_PITCH,
                             -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);
 
    float rOut=updatePID(pidRoll, rollRateSP,  gyroRoll, dt, tpa, agBoost);
    float pOut=updatePID(pidPitch,pitchRateSP,gyroPitch, dt, tpa, agBoost);
    float yOut=updatePID(pidYaw,  yawRateFilt,  gyroYaw, dt, 1.0f, 1.0f);
 
    float pm=-pOut*MIX_PITCH, rm=rOut*MIX_ROLL, ym=yOut*MIX_YAW;
 
    float fFL=thr+pm+rm-ym;
    float fFR=thr+pm-rm+ym;
    float fBR=thr-pm-rm-ym;
    float fBL=thr-pm+rm+ym;
 
    mFLf=lpf(mFLf,fFL,MOTOR_LPF); mFRf=lpf(mFRf,fFR,MOTOR_LPF);
    mBRf=lpf(mBRf,fBR,MOTOR_LPF); mBLf=lpf(mBLf,fBL,MOTOR_LPF);
 
    motorFL.writeMicroseconds(armed((int)roundf(mFLf)));
    motorFR.writeMicroseconds(armed((int)roundf(mFRf)));
    motorBR.writeMicroseconds(armed((int)roundf(mBRf)));
    motorBL.writeMicroseconds(armed((int)roundf(mBLf)));
}