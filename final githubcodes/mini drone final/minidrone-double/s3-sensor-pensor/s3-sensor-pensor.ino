

#include <Wire.h>
#include <SPI.h>
#include "ICM_20948.h"
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BMP280.h>
#include "Bitcraze_PMW3901.h"

// ── Pin definitions ───────────────────────────────────────────
#define SDA_PIN      5
#define SCL_PIN      6
#define XSHUT_PIN   -1     // No XSHUT pin connected — XSHUT tied to 3.3V directly
#define PMW_CS_PIN  44
#define PMW_SCK_PIN  7
#define PMW_MISO_PIN 8
#define PMW_MOSI_PIN 9
#define UART_TX_PIN 43     // → C3 UART RX

// ── Packet sent to C3 every ~5 ms ─────────────────────────────
// Keep this struct IDENTICAL in both sketches
#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  header;      // always 0xAA — sync byte
  float    roll;        // deg, +ve = right wing down
  float    pitch;       // deg, +ve = nose up
  float    gyrX;        // dps
  float    gyrY;        // dps
  float    gyrZ;        // dps
  float    heightMM;    // fused height above ground, mm
  float    velZ;        // vertical speed, mm/s (+ve = climbing)
  int16_t  flowX;       // optical flow, pixel/s × 10
  int16_t  flowY;
  uint8_t  valid;       // bitmask: bit0=IMU  bit1=height  bit2=flow
  uint8_t  checksum;    // XOR of bytes [1 … sizeof-2]
};
#pragma pack(pop)
// struct is 35 bytes — fits in one UART frame in ~0.6 ms @ 500 kbaud

// ── Sensor objects ────────────────────────────────────────────
ICM_20948_I2C    icm;
Adafruit_VL53L1X vl53;
Adafruit_BMP280  bmp;
Bitcraze_PMW3901 flow(PMW_CS_PIN);

bool icmOK = false, vl53OK = false, bmpOK = false, flowOK = false;

// ── Filter / fusion state ─────────────────────────────────────
float cf_roll  = 0.0f, cf_pitch = 0.0f;   // complementary filter
float heightMM = 0.0f, velZ = 0.0f;
float bmpBiasMM  = 0.0f;
float bmpRelAltMM = 0.0f;    // latest BMP relative altitude (always updated)
bool  bmpBiasSet = false;


const float VL53_ENGAGE_MM    = 150.0f;  // BMP alt to start trusting VL53
const float VL53_DISENGAGE_MM =  80.0f;  // drop back below this → revert to BMP
const float VL53_AGREE_BASE   = 200.0f;  // base cross-validation window (mm)
const float VL53_AGREE_VELSCL =   0.08f; // +Nmm per mm/s velZ (widens when moving fast)
const int   VL53_MAX_REJECTS  =    4;    // consecutive disagreements before temp disengaging
bool vl53Active = false;
int  vl53RejectCount = 0;       // consecutive BMP-disagreement counter
int  vl53AcceptCount = 0;       // consecutive good readings (used for re-engagement)

unsigned long lastLoopUs  = 0;
unsigned long lastBmpMs   = 0;
unsigned long lastDebugMs = 0;

// ── Checksum ──────────────────────────────────────────────────
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 1; i < len - 1; i++) cs ^= data[i];
  return cs;
}

// ── I2C bus scanner ───────────────────────────────────────────
void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", a);
      if (a == 0x29)              Serial.print("  ← VL53L1X");
      if (a == 0x68 || a == 0x69) Serial.print("  ← ICM-20948");
      if (a == 0x76 || a == 0x77) Serial.print("  ← BMP280");
      Serial.println();
      found++;
    }
    delay(5);
  }
  Serial.printf("  %d device(s)\n\n", found);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== S3 SENSE — SENSOR NODE ===");

  // UART to C3 (TX only, no RX needed)
  Serial1.begin(500000, SERIAL_8N1, -1, UART_TX_PIN);

  // Pull XSHUT high so VL53L1X powers on
  if (XSHUT_PIN >= 0) {
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);
  }

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  scanI2C();

  // ── ICM-20948 ──
  Serial.print("ICM-20948... ");
  icmOK = (icm.begin(Wire, 0x68) == ICM_20948_Stat_Ok);
  if (!icmOK) icmOK = (icm.begin(Wire, 0x69) == ICM_20948_Stat_Ok);
  Serial.println(icmOK ? "OK" : "NOT FOUND");

  // ── VL53L1X ──
  Serial.print("VL53L1X...  ");
  if (vl53.begin(0x29, &Wire)) {
    vl53.setTimingBudget(50);
    vl53.startRanging();
    vl53OK = true;
  }
  Serial.println(vl53OK ? "OK" : "NOT FOUND");

  // ── BMP280 ──
  Serial.print("BMP280...   ");
  if      (bmp.begin(0x76)) bmpOK = true;
  else if (bmp.begin(0x77)) bmpOK = true;
  if (bmpOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );
  }
  Serial.println(bmpOK ? "OK" : "NOT FOUND");

  // ── PMW3901 ──
  Serial.print("PMW3901...  ");
  SPI.begin(PMW_SCK_PIN, PMW_MISO_PIN, PMW_MOSI_PIN, PMW_CS_PIN);
  flowOK = flow.begin();
  Serial.println(flowOK ? "OK" : "NOT FOUND");

  Serial.printf("\n[IMU:%s] [ToF:%s] [Baro:%s] [Flow:%s]\n\n",
    icmOK  ? "OK" : "--",
    vl53OK ? "OK" : "--",
    bmpOK  ? "OK" : "--",
    flowOK ? "OK" : "--");

  lastLoopUs = micros();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  if (dt < 0.005f) return;   // 200 Hz cap
  lastLoopUs = nowUs;

  SensorPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.header = 0xAA;

  // ── IMU: complementary filter for roll/pitch ──────────────
  if (icmOK && icm.dataReady()) {
    icm.getAGMT();

    float ax = icm.accX() / 1000.0f;   // g
    float ay = icm.accY() / 1000.0f;
    float az = icm.accZ() / 1000.0f;
    float gx = icm.gyrX();              // dps
    float gy = icm.gyrY();
    float gz = icm.gyrZ();

    // Accel angles (fallback reference)
    // cf_roll  integrates gy / references accel_pitch  ← physical roll axis
    // cf_pitch integrates gx / references accel_roll   ← physical pitch axis
    float accel_roll  =  atan2f(ay, az)                     * 57.29578f;
    float accel_pitch =  atan2f(-ax, sqrtf(ay*ay + az*az))  * 57.29578f;

    // Complementary filter — 98 % gyro, 2 % accel
    // Axes swapped to match physical drone orientation
    const float alpha = 0.98f;
    cf_roll  = alpha * (cf_roll  + gy * dt) + (1.0f - alpha) * accel_pitch;
    cf_pitch = alpha * (cf_pitch + gx * dt) + (1.0f - alpha) * accel_roll;

    pkt.roll  = cf_roll;
    pkt.pitch = cf_pitch;
    pkt.gyrX  = gy;    // roll rate (physical X)
    pkt.gyrY  = gx;    // pitch rate (physical Y)
    pkt.gyrZ  = gz;
    pkt.valid |= 0x01;
  }

  // ── Height fusion ─────────────────────────────────────────────
  float newHeight = -1.0f;

  // ── BMP280 always sampled at ~10 Hz ──────────────────────────
  if (bmpOK) {
    unsigned long nowMs = millis();
    if (nowMs - lastBmpMs > 100) {
      lastBmpMs = nowMs;
      float bmpAlt = bmp.readAltitude(1013.25f) * 1000.0f;
      if (!bmpBiasSet) {
        bmpBiasMM  = bmpAlt;   // first reading = ground reference
        bmpBiasSet = true;
      }
      bmpRelAltMM = bmpAlt - bmpBiasMM;

      // Update VL53 engagement flag from BMP altitude
      if (!vl53Active && bmpRelAltMM > VL53_ENGAGE_MM) {
        vl53Active = true;
      } else if (vl53Active && bmpRelAltMM < VL53_DISENGAGE_MM) {
        vl53Active = false;   // back near ground — stop trusting VL53
      }
    }
  }

  // ── VL53L1X — cross-validated against BMP ────────────────────
  if (vl53Active && vl53OK && vl53.dataReady()) {
    int16_t d = vl53.distance();
    vl53.clearInterrupt();

    if (d > 30 && d < 3000) {
      float vl53mm = (float)d;

      // ── Check 1: cycle-to-cycle spike filter ────────────────
      bool spikeOK = (heightMM <= 0.0f || fabsf(vl53mm - heightMM) < 250.0f);

      // ── Check 2: BMP cross-validation ───────────────────────
      // Tolerance widens when moving fast (BMP lags real motion)
      float tolerance = VL53_AGREE_BASE + fabsf(velZ) * VL53_AGREE_VELSCL;
      float bmpDiff   = fabsf(vl53mm - bmpRelAltMM);
      bool  bmpAgrees = !bmpBiasSet || (bmpDiff < tolerance);

      if (spikeOK && bmpAgrees) {
        // ── Both checks passed — accept reading ───────────────
        newHeight     = vl53mm;
        vl53RejectCount = 0;
        vl53AcceptCount++;

        // Keep BMP bias aligned so it stays in sync with VL53
        if (bmpOK && bmpBiasSet) {
          // Soft-blend the bias correction to avoid jumps
          float bmpAlt  = bmp.readAltitude(1013.25f) * 1000.0f;
          bmpBiasMM     = bmpBiasMM * 0.98f + (bmpAlt - vl53mm) * 0.02f;
        }

      } else {
        // ── Disagreement — reject this reading ────────────────
        vl53RejectCount++;
        vl53AcceptCount = 0;

        if (vl53RejectCount >= VL53_MAX_REJECTS) {
          // Sustained bad readings → temporarily disengage VL53,
          // fall back to BMP until BMP confirms we're airborne again
          vl53Active      = false;
          vl53RejectCount = 0;
          // (re-engagement happens in the BMP block above when
          //  bmpRelAltMM rises back above VL53_ENGAGE_MM)
        }
        // newHeight stays -1 → BMP fallback below will fill it
      }
    } else {
      // Out of range reading
      vl53RejectCount++;
    }
  }

  // ── BMP fallback if VL53 not engaged or reading bad ───────────
  if (newHeight < 0.0f && bmpBiasSet) {
    newHeight = bmpRelAltMM;
  }

  // ── Velocity — spike-clamped derivative ───────────────────────
  if (newHeight >= 0.0f) {
    if (heightMM > 0.0f) {
      // Clamp raw delta to ±500mm/cycle to kill sensor spike noise
      float deltaH   = constrain(newHeight - heightMM, -500.0f, 500.0f);
      float rawVelZ  = (dt > 1e-4f) ? deltaH / dt : 0.0f;
      // Clamp to physically plausible vertical speed (±3 m/s)
      rawVelZ        = constrain(rawVelZ, -3000.0f, 3000.0f);
      velZ           = velZ * 0.80f + rawVelZ * 0.20f;
    }
    heightMM = newHeight;
    pkt.valid |= 0x02;
  }

  pkt.heightMM = heightMM;
  pkt.velZ     = velZ;

  // ── Optical flow ──────────────────────────────────────────
  if (flowOK) {
    int16_t fx = 0, fy = 0;
    flow.readMotionCount(&fx, &fy);
    pkt.flowX = (int16_t)(fx * 10);
    pkt.flowY = (int16_t)(fy * 10);
    if (fx != 0 || fy != 0) pkt.valid |= 0x04;
  }

  // ── Send packet ───────────────────────────────────────────
  pkt.checksum = calcChecksum((uint8_t*)&pkt, sizeof(pkt));
  Serial1.write((uint8_t*)&pkt, sizeof(pkt));

  // ── Debug at 10 Hz to USB serial ─────────────────────────
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs > 100) {
    lastDebugMs = nowMs;
    Serial.printf(
      "R:%+6.1f P:%+6.1f | H:%5.0fmm Vz:%+5.0f | BMP:%+5.0f diff:%4.0f [%s ok:%d rej:%d] | gX:%+5.1f gY:%+5.1f gZ:%+5.1f | v:0x%02X\n",
      pkt.roll, pkt.pitch,
      pkt.heightMM, pkt.velZ,
      bmpRelAltMM, fabsf(pkt.heightMM - bmpRelAltMM),
      vl53Active ? "VL53" : "BMP ",
      vl53AcceptCount, vl53RejectCount,
      pkt.gyrX, pkt.gyrY, pkt.gyrZ,
      pkt.valid);
  }
}