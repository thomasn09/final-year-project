
// All pins to test
const int PINS[] = { 12, 13, 14, 16, 17, 25, 26, 27, 32, 33 };
const int NUM_PINS = sizeof(PINS) / sizeof(PINS[0]);
 
int lastState[NUM_PINS];
 
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BUTTON TEST ===");
  Serial.println("Press each button/flip each switch one at a time\n");
 
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    lastState[i] = digitalRead(PINS[i]);
  }
 
  // Print header
  Serial.print("Monitoring pins: ");
  for (int i = 0; i < NUM_PINS; i++) {
    Serial.print(PINS[i]);
    if (i < NUM_PINS - 1) Serial.print(", ");
  }
  Serial.println("\n");
}
 
void loop() {
  for (int i = 0; i < NUM_PINS; i++) {
    int state = digitalRead(PINS[i]);
 
    if (state != lastState[i]) {
      delay(20);  // debounce
      state = digitalRead(PINS[i]);
      if (state != lastState[i]) {
        lastState[i] = state;
        Serial.printf("GPIO %-2d  →  %s\n",
          PINS[i],
          state == LOW ? "PRESSED / ON" : "RELEASED / OFF");
      }
    }
  }
}