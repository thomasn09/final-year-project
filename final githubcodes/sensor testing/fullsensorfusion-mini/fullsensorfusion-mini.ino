
#include <Wire.h>
#include <SPI.h>
#include "ICM_20948.h"
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BMP280.h>
#include <Bitcraze_PMW3901.h>

// =========================
// I2C pins
// =========================
#define SDA_PIN 5
#define SCL_PIN 6


#define XSHUT_PIN 4


static const int PIN_PMW_CS   = 44;
static const int PIN_PMW_INT  = 5;   // optional, not used
static const int PIN_SPI_SCK  = 7;
static const int PIN_SPI_MISO = 8;
static const int PIN_SPI_MOSI = 9;


ICM_20948_I2C    icm;
Adafruit_VL53L1X vl53;
Adafruit_BMP280  bmp;
Bitcraze_PMW3901 flow(PIN_PMW_CS);


bool icmOK  = false;
bool vl53OK = false;
bool bmpOK  = false;
bool flowOK = false;


int16_t dx = 0, dy = 0;
long totalX = 0;
long totalY = 0;
unsigned long lastFlowPrint = 0;


float countsToCmX = 0.05f;
float countsToCmY = 0.05f;


void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X", addr);
      if (addr == 0x29) Serial.print("  <- VL53L1X");
      if (addr == 0x68 || addr == 0x69) Serial.print("  <- ICM-20948");
      if (addr == 0x76 || addr == 0x77) Serial.print("  <- BMP280");
      Serial.println();
      found++;
    }
    delay(5);
  }

  if (found == 0) Serial.println("  No devices found - check wiring!");
  else Serial.printf("  %d device(s) found\n", found);

  Serial.println();
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n================================");
  Serial.println("   SENSOR TEST - MINI DRONE");
  Serial.println("================================\n");

  // Pull XSHUT HIGH before starting I2C
  if (XSHUT_PIN >= 0) {
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);
  }

  // I2C init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Scan I2C bus
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
  Serial.print("VL53L1X (0x29)... ");
  if (vl53.begin(0x29, &Wire)) {
    vl53.setTimingBudget(50);
    vl53.startRanging();
    vl53OK = true;
    Serial.println("OK");
  } else {
    Serial.println("NOT FOUND");
  }

  // ── BMP280 ──
  Serial.print("BMP280 (0x76/77)... ");
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

  // ── PMW3901 ──
  Serial.println();
  Serial.println("PMW3901 optical flow init...");

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_PMW_CS);

  pinMode(PIN_PMW_CS, OUTPUT);
  digitalWrite(PIN_PMW_CS, HIGH);

  if (!flow.begin()) {
    Serial.println("ERROR: PMW3901 init failed.");
    Serial.println("Check wiring:");
    Serial.println("  SCK  -> GPIO7");
    Serial.println("  MISO -> GPIO8");
    Serial.println("  MOSI -> GPIO9");
    Serial.println("  CS   -> GPIO1");
    Serial.println("  VCC  -> 3.3V or 5V (module dependent)");
    Serial.println("  GND  -> GND");
  } else {
    flowOK = true;
    Serial.println("PMW3901 initialized OK");
    Serial.println("Move the board over a textured surface.");
  }

  Serial.println();
  Serial.printf("══ ICM-20948[%s]  VL53L1X[%s]  BMP280[%s]  PMW3901[%s] ══\n\n",
    icmOK  ? "OK" : "XX",
    vl53OK ? "OK" : "XX",
    bmpOK  ? "OK" : "XX",
    flowOK ? "OK" : "XX"
  );

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
        icm.accX() / 1000.0f,
        icm.accY() / 1000.0f,
        icm.accZ() / 1000.0f);
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
    Serial.printf("BMP  | Temp: %.1f C   Pressure: %.2f hPa   Alt: %.2f m\n",
      bmp.readTemperature(),
      bmp.readPressure() / 100.0f,
      bmp.readAltitude(1013.25f));
  } else {
    Serial.println("BMP  | NOT CONNECTED");
  }

  // PMW3901
  if (flowOK) {
    flow.readMotionCount(&dx, &dy);

    totalX += dx;
    totalY += dy;

    if (millis() - lastFlowPrint >= 50) {
      lastFlowPrint = millis();

      float estCmX = totalX * countsToCmX;
      float estCmY = totalY * countsToCmY;

      Serial.print("FLOW | dx: ");
      Serial.print(dx);
      Serial.print("\t dy: ");
      Serial.print(dy);

      Serial.print("\t totalX: ");
      Serial.print(totalX);
      Serial.print("\t totalY: ");
      Serial.print(totalY);

      Serial.print("\t estX_cm: ");
      Serial.print(estCmX, 2);
      Serial.print("\t estY_cm: ");
      Serial.println(estCmY, 2);
    }
  } else {
    Serial.println("FLOW | NOT CONNECTED");
  }

  Serial.println();
  delay(200);
}