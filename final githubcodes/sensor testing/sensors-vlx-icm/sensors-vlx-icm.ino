
 
#include <Wire.h>
#include "ICM_20948.h"
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BMP280.h>
 
#define SDA_PIN 5
#define SCL_PIN 6
 
ICM_20948_I2C    icm;
Adafruit_VL53L1X vl53;
Adafruit_BMP280  bmp;
 
bool icmOK  = false;
bool vl53OK = false;
bool bmpOK  = false;
 

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X", addr);
      // Annotate known sensors
      if (addr == 0x29)              Serial.print("  ← VL53L1X");
      if (addr == 0x68 || addr == 0x69) Serial.print("  ← ICM-20948");
      if (addr == 0x76 || addr == 0x77) Serial.print("  ← BMP280");
      Serial.println();
      found++;
    }
    delay(5);
  }
  if (found == 0) Serial.println("  No devices found — check wiring!");
  else Serial.printf("  %d device(s) found\n", found);
  Serial.println();
}
 
// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================================");
  Serial.println("   SENSOR TEST — MINI DRONE");
  Serial.println("================================\n");
 
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
 
  // Scan first — shows exactly what's visible on the bus
  scanI2C();
 
  // ── ICM-20948 ──
  Serial.print("ICM-20948 (0x68)... ");
  if (icm.begin(Wire, 0x68) == ICM_20948_Stat_Ok) {
    icmOK = true;
    Serial.println("OK");
  } else {
    Serial.print("not at 0x68, trying 0x69... ");
    if (icm.begin(Wire, 0x69) == ICM_20948_Stat_Ok) {
      icmOK = true;
      Serial.println("OK at 0x69");
    } else {
      Serial.println("NOT FOUND");
    }
  }
 
  // ── VL53L1X ──
  Serial.print("VL53L1X  (0x29)... ");
  if (vl53.begin(0x29, &Wire)) {
    vl53.setTimingBudget(50);
    vl53.startRanging();
    vl53OK = true;
    Serial.println("OK");
  } else {
    Serial.println("NOT FOUND");
  }
 
  // ── BMP280 ──
  Serial.print("BMP280   (0x76/77)... ");
  if (bmp.begin(0x76)) {
    bmpOK = true;
    Serial.println("OK at 0x76");
  } else if (bmp.begin(0x77)) {
    bmpOK = true;
    Serial.println("OK at 0x77");
  } else {
    Serial.println("NOT FOUND");
  }
 
  if (bmpOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );
  }
 
  Serial.println();
  Serial.printf("══ ICM-20948[%s]  VL53L1X[%s]  BMP280[%s] ══\n\n",
    icmOK  ? "OK" : "XX",
    vl53OK ? "OK" : "XX",
    bmpOK  ? "OK" : "XX");
 
  delay(500);
}
 
// ── Loop ─────────────────────────────────────────────────────────
void loop() {
  Serial.println("────────────────────────────────");
 
  // ICM-20948
  if (icmOK) {
    if (icm.dataReady()) {
      icm.getAGMT();
      Serial.printf("ICM  | Accel  X:%7.3f  Y:%7.3f  Z:%7.3f g\n",
        icm.accX()/1000.0f, icm.accY()/1000.0f, icm.accZ()/1000.0f);
      Serial.printf("     | Gyro   X:%7.2f  Y:%7.2f  Z:%7.2f dps\n",
        icm.gyrX(), icm.gyrY(), icm.gyrZ());
    } else {
      Serial.println("ICM  | waiting for data...");
    }
  } else {
    Serial.println("ICM  | NOT CONNECTED");
  }
 
  // VL53L1X
  if (vl53OK) {
    if (vl53.dataReady()) {
      int16_t dist = vl53.distance();
      vl53.clearInterrupt();
      if (dist > 0)
        Serial.printf("VL53 | Distance: %d mm\n", dist);
      else
        Serial.println("VL53 | Out of range");
    } else {
      Serial.println("VL53 | measuring...");
    }
  } else {
    Serial.println("VL53 | NOT CONNECTED");
  }
 
  // BMP280
  if (bmpOK) {
    Serial.printf("BMP  | Temp: %.1f°C   Pressure: %.2f hPa   Alt: %.2f m\n",
      bmp.readTemperature(),
      bmp.readPressure() / 100.0f,
      bmp.readAltitude(1013.25f));
  } else {
    Serial.println("BMP  | NOT CONNECTED");
  }
 
  Serial.println();
  delay(500);
}
 