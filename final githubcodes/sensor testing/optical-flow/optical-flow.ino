#include <SPI.h>
#include <Bitcraze_PMW3901.h>


static const int PIN_PMW_CS   = 1;   

static const int PIN_SPI_SCK  = 7;
static const int PIN_SPI_MISO = 8;
static const int PIN_SPI_MOSI = 9;

// ---------- Optical flow sensor ----------
Bitcraze_PMW3901 flow(PIN_PMW_CS);

// ---------- Motion accumulation ----------
int16_t dx = 0, dy = 0;
long totalX = 0;
long totalY = 0;

// ---------- Timing ----------
unsigned long lastPrint = 0;


float countsToCmX = 0.05f;  
float countsToCmY = 0.05f;  

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("PMW3901 optical flow test on ESP32-S3 Sense");

  // Start SPI on the ESP32-S3 pins you selected
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_PMW_CS);

  pinMode(PIN_PMW_CS, OUTPUT);
  digitalWrite(PIN_PMW_CS, HIGH);

  // Initialize flow sensor
  if (!flow.begin()) {
    Serial.println("ERROR: PMW3901 init failed.");
    Serial.println("Check wiring:");
    Serial.println("  SCK  -> GPIO7");
    Serial.println("  MISO -> GPIO8");
    Serial.println("  MOSI -> GPIO9");
    Serial.println("  CS   -> GPIO4");
    Serial.println("  VCC  -> 5V");
    Serial.println("  GND  -> GND");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("PMW3901 initialized OK");
  Serial.println("Move the board over a textured surface.");
  Serial.println("---------------------------------------------------");
}

void loop() {
  // Read motion since last call
  flow.readMotionCount(&dx, &dy);

  // Accumulate position-like estimate
  totalX += dx;
  totalY += dy;

  // Print at 20 Hz
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();

    float estCmX = totalX * countsToCmX;
    float estCmY = totalY * countsToCmY;

    Serial.print("dx: ");
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
}