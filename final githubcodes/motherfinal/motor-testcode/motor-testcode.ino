/*
 * Motor Direction Test  (with BLHeli ESC calibration on boot)
 *
 * On startup it runs the ESC calibration sequence automatically:
 *   1. Sends MAX (2000us) — plug in LiPo during this 8-second window
 *   2. Sends MIN (1000us) — ESCs beep and save the range
 *
 * After calibration it spins each motor for 3 seconds so you can
 * check which direction each one spins.
 *
 * For a standard X-frame quad the correct directions are:
 *
 *        FRONT
 *   FL  (CCW)  |  FR  (CW)
 *   ---------- + ----------
 *   BL  (CW)   |  BR  (CCW)
 *
 * If a motor spins the wrong way, swap any TWO of its three
 * phase wires at the ESC.
 *
 * REMOVE PROPS BEFORE RUNNING
 */
 
#include <ESP32Servo.h>
 
// ================================================================
// PINS — back two swapped
// ================================================================
#define MOTOR_FL_PIN 15
#define MOTOR_FR_PIN  2
#define MOTOR_BR_PIN 16   // swapped
#define MOTOR_BL_PIN 17   // swapped
 
#define TEST_SPEED 1250   // safe low speed — raise if motors don't spin
#define SPIN_TIME  3000   // ms each motor spins for
 
Servo motorFL, motorFR, motorBR, motorBL;
 
void stopAll() {
    motorFL.writeMicroseconds(1000);
    motorFR.writeMicroseconds(1000);
    motorBR.writeMicroseconds(1000);
    motorBL.writeMicroseconds(1000);
}
 
void setup() {
    Serial.begin(115200);
    delay(500);
 
    motorFL.setPeriodHertz(50); motorFL.attach(MOTOR_FL_PIN, 1000, 2000);
    motorFR.setPeriodHertz(50); motorFR.attach(MOTOR_FR_PIN, 1000, 2000);
    motorBR.setPeriodHertz(50); motorBR.attach(MOTOR_BR_PIN, 1000, 2000);
    motorBL.setPeriodHertz(50); motorBL.attach(MOTOR_BL_PIN, 1000, 2000);
 
    // ================================================================
    // BLHeli ESC CALIBRATION
    // ================================================================
    Serial.println("================================");
    Serial.println(" ESC CALIBRATION");
    Serial.println("================================");
    Serial.println(" Step 1: UNPLUG battery / ESC power");
    Serial.println("         Keep USB plugged in");
    Serial.println("         Sending MAX signal in 3s...");
    delay(3000);
 
    Serial.println();
    Serial.println(" Sending MAX (2000us) now.");
    Serial.println(" >>> PLUG IN YOUR LIPO NOW <<<");
    Serial.println(" You have 8 seconds.");
    Serial.println(" Listen for: beep-beep (high-low tone)");
    motorFL.writeMicroseconds(2000);
    motorFR.writeMicroseconds(2000);
    motorBR.writeMicroseconds(2000);
    motorBL.writeMicroseconds(2000);
 
    for (int i = 8; i > 0; i--) {
        Serial.printf(" %d...\n", i);
        delay(1000);
    }
 
    Serial.println();
    Serial.println(" Sending MIN (1000us) now.");
    Serial.println(" Listen for: long beep or melody = CALIBRATION SAVED");
    motorFL.writeMicroseconds(1000);
    motorFR.writeMicroseconds(1000);
    motorBR.writeMicroseconds(1000);
    motorBL.writeMicroseconds(1000);
 
    for (int i = 8; i > 0; i--) {
        Serial.printf(" %d...\n", i);
        delay(1000);
    }
 
    Serial.println(" Calibration complete!");
    Serial.println("================================");
    Serial.println();
 
    // Hold MIN for a moment so ESCs arm before direction test
    stopAll();
    delay(2000);
 
    Serial.println("================================");
    Serial.println(" MOTOR DIRECTION TEST");
    Serial.println(" PROPS OFF");
    Serial.println("================================");
    Serial.println(" Correct directions for X-frame:");
    Serial.println("   FL = CCW (anti-clockwise)");
    Serial.println("   FR = CW  (clockwise)");
    Serial.println("   BR = CCW (anti-clockwise)");
    Serial.println("   BL = CW  (clockwise)");
    Serial.println();
    Serial.println(" If wrong: swap any 2 of the 3");
    Serial.println(" motor phase wires on that ESC");
    Serial.println("================================");
    Serial.println();
 
    // ---- FL ----
    Serial.println(">>> FRONT LEFT (FL) spinning...");
    Serial.println("    Expected: CCW (anti-clockwise)");
    motorFL.writeMicroseconds(TEST_SPEED);
    delay(SPIN_TIME);
    stopAll();
    Serial.println("    Stopped. Note direction above.");
    Serial.println();
    delay(1500);
 
    // ---- FR ----
    Serial.println(">>> FRONT RIGHT (FR) spinning...");
    Serial.println("    Expected: CW (clockwise)");
    motorFR.writeMicroseconds(TEST_SPEED);
    delay(SPIN_TIME);
    stopAll();
    Serial.println("    Stopped. Note direction above.");
    Serial.println();
    delay(1500);
 
    // ---- BR ----
    Serial.println(">>> BACK RIGHT (BR) spinning...");
    Serial.println("    Expected: CCW (anti-clockwise)");
    motorBR.writeMicroseconds(TEST_SPEED);
    delay(SPIN_TIME);
    stopAll();
    Serial.println("    Stopped. Note direction above.");
    Serial.println();
    delay(1500);
 
    // ---- BL ----
    Serial.println(">>> BACK LEFT (BL) spinning...");
    Serial.println("    Expected: CW (clockwise)");
    motorBL.writeMicroseconds(TEST_SPEED);
    delay(SPIN_TIME);
    stopAll();
    Serial.println("    Stopped. Note direction above.");
    Serial.println();
    delay(1500);
 
    // ---- All together ----
    Serial.println(">>> ALL MOTORS spinning...");
    Serial.println("    Check all 4 spin at same speed");
    motorFL.writeMicroseconds(TEST_SPEED);
    motorFR.writeMicroseconds(TEST_SPEED);
    motorBR.writeMicroseconds(TEST_SPEED);
    motorBL.writeMicroseconds(TEST_SPEED);
    delay(SPIN_TIME);
    stopAll();
    Serial.println("    Done.");
    Serial.println();
 
    Serial.println("================================");
    Serial.println(" TEST COMPLETE");
    Serial.println(" Type a command to retest:");
    Serial.println("   1 = FL   2 = FR");
    Serial.println("   3 = BR   4 = BL");
    Serial.println("   a = ALL  s = STOP");
    Serial.println("================================");
}
 
void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        stopAll();
        delay(200);
        switch (c) {
            case '1':
                Serial.println("FL spinning — should be CCW");
                motorFL.writeMicroseconds(TEST_SPEED);
                delay(SPIN_TIME);
                stopAll();
                break;
            case '2':
                Serial.println("FR spinning — should be CW");
                motorFR.writeMicroseconds(TEST_SPEED);
                delay(SPIN_TIME);
                stopAll();
                break;
            case '3':
                Serial.println("BR spinning — should be CCW");
                motorBR.writeMicroseconds(TEST_SPEED);
                delay(SPIN_TIME);
                stopAll();
                break;
            case '4':
                Serial.println("BL spinning — should be CW");
                motorBL.writeMicroseconds(TEST_SPEED);
                delay(SPIN_TIME);
                stopAll();
                break;
            case 'a':
            case 'A':
                Serial.println("ALL spinning");
                motorFL.writeMicroseconds(TEST_SPEED);
                motorFR.writeMicroseconds(TEST_SPEED);
                motorBR.writeMicroseconds(TEST_SPEED);
                motorBL.writeMicroseconds(TEST_SPEED);
                delay(SPIN_TIME);
                stopAll();
                break;
            case 's':
            case 'S':
                Serial.println("Stopped");
                break;
        }
    }
}