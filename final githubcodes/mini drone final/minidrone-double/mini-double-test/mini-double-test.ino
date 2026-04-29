
#define FL 3
#define FR 2
#define BL 4
#define BR 5

#define MIN_US   1000
#define MAX_US   2000
#define PWM_FREQ   50
#define PWM_RES    16

// ── helpers ───────────────────────────────────────────────────────────────────
uint32_t usToDuty(int us) {
  return (uint32_t)(us / 20000.0f * 65535.0f);
}

void writePin(int pin, int us) {
  ledcWrite(pin, usToDuty(us));
}

void allPins(int us) {
  writePin(FL, us);
  writePin(FR, us);
  writePin(BL, us);
  writePin(BR, us);
}

void waitForEnter(const char* msg) {
  Serial.println(msg);
  Serial.println("  --> Press ENTER in Serial Monitor to continue...");
  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(FL, PWM_FREQ, PWM_RES);
  ledcAttach(FR, PWM_FREQ, PWM_RES);
  ledcAttach(BL, PWM_FREQ, PWM_RES);
  ledcAttach(BR, PWM_FREQ, PWM_RES);
  allPins(MIN_US);

  Serial.println("\n\n=================================================");
  Serial.println("  MOTOR DIAGNOSTIC — ESP32-C3");
  Serial.println("=================================================");
  Serial.println("Before starting, confirm:");
  Serial.println("  1. PROPS ARE OFF");
  Serial.println("  2. ESC signal wires are on GPIO 4/5/6/10");
  Serial.println("  3. ESC GND wire shares ground with C3 GND");
  Serial.println("  4. LiPo is DISCONNECTED right now");
  Serial.println();

  // ── STEP 1: PWM sanity check ────────────────────────────────────────────────
  waitForEnter("STEP 1 — PWM output check.");
  Serial.println("Outputting 1000 us MIN on all four pins.");
  Serial.println("If you have an oscilloscope or servo tester, verify 1ms pulses at 50Hz.");
  Serial.println("If not, just continue.");
  allPins(MIN_US);
  delay(3000);

  // ── STEP 2: calibration with battery connect prompt ─────────────────────────
  waitForEnter("STEP 2 — ESC calibration. BATTERY STILL DISCONNECTED.");
  Serial.println("Sending MAX (2000 us)...");
  allPins(MAX_US);
  Serial.println("  --> Connect the LiPo battery NOW. You have 5 seconds.");

  // Count down so user can see progress
  for (int i = 5; i > 0; i--) {
    Serial.printf("  %d...\n", i);
    delay(1000);
  }

  Serial.println("Dropping to MIN (1000 us)...");
  allPins(MIN_US);
  Serial.println("  --> Listen for ESC beeps confirming range saved.");
  Serial.println("  Waiting 5 seconds...");
  delay(5000);

  // ── STEP 3: arm wait ────────────────────────────────────────────────────────
  Serial.println("Holding MIN for 3 more seconds to allow ESCs to arm...");
  delay(3000);
  Serial.println("ESCs should be armed now.");

  // ── STEP 4: spin ALL together at increasing throttle ────────────────────────
  waitForEnter("STEP 3 — Spin test. REMOVE PROPS NOW if not already done.");
  int levels[] = { 1100, 1150, 1200, 1300, 1400 };
  int numLevels = 5;

  for (int i = 0; i < numLevels; i++) {
    int us = levels[i];
    Serial.printf("Trying ALL motors at %d us for 2 seconds...\n", us);
    allPins(us);
    delay(2000);
    allPins(MIN_US);
    Serial.println("  Stopped. Did any motor spin? Note which ones.");
    delay(1000);
  }

  // ── STEP 4: individual pin test ─────────────────────────────────────────────
  waitForEnter("STEP 4 — Individual motor test at 1300 us.");
  int pins[]          = { FL,            FR,             BL,            BR };
  const char* names[] = { "FRONT LEFT", "FRONT RIGHT", "BACK LEFT",  "BACK RIGHT" };

  for (int i = 0; i < 4; i++) {
    Serial.printf(">>> %s (GPIO%d) — spinning now...\n", names[i], pins[i]);
    writePin(pins[i], 1300);
    delay(2000);
    writePin(pins[i], MIN_US);
    Serial.printf("    Stopped. Did GPIO%d motor spin? (y/n)\n", pins[i]);
    delay(1000);
  }

  Serial.println("\n=================================================");
  Serial.println("DIAGNOSTIC COMPLETE.");
  Serial.println("If NO motors spun at any level:");
  Serial.println("  → Most likely: ESC GND not connected to C3 GND");
  Serial.println("  → Or: LiPo not connected when MAX was sent");
  Serial.println("  → Or: ESC signal wire on wrong GPIO");
  Serial.println();
  Serial.println("If SOME motors spun but not others:");
  Serial.println("  → Bad solder joint on the silent ESC signal wire");
  Serial.println("  → That ESC may need its own calibration run");
  Serial.println();
  Serial.println("If motors spin here but not in flight code:");
  Serial.println("  → ESC calibration missing from flight code setup()");
  Serial.println("=================================================");
  Serial.println("All outputs now at MIN. Safe to disconnect.");
  allPins(MIN_US);
}

void loop() {
  // Nothing — diagnostic is done in setup
}