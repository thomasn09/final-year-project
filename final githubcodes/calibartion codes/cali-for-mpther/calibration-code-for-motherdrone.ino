/*
 * BLHeli ESC Calibration + Motor Test — ESP32
 *
 * USE THIS TO:
 *   1. Calibrate all 4 ESCs so they share the same throttle range
 *   2. Test each motor individually to confirm spin direction and balance
 *
 * !! REMOVE ALL PROPELLERS BEFORE RUNNING !!
 *
 * PINS (big drone):
 *   FL = 15, FR = 2, BR = 17, BL = 16
 *
 * HOW BLHeli CALIBRATION WORKS:
 *   - ESC must see MAX signal when it powers on → it stores the high point
 *   - Then you send MIN signal → it stores the low point
 *   - Both points are saved to ESC memory
 *   - After calibration all ESCs respond identically to the same PWM value
 */
 
#include <ESP32Servo.h>
 
// ================================================================
// PINS — adjust if needed
// ================================================================
#define MOTOR_FL_PIN 15
#define MOTOR_FR_PIN  2
#define MOTOR_BR_PIN 17
#define MOTOR_BL_PIN 16
 
// ================================================================
// PWM VALUES
// ================================================================
#define CAL_MAX_US   2000   // calibration high point
#define CAL_MIN_US   1000   // calibration low point (absolute min for BLHeli)
#define FLIGHT_MIN   1100   // your normal disarmed value
#define FLIGHT_MAX   2000   // your normal max
 
// ================================================================
// MOTORS
// ================================================================
Servo motorFL, motorFR, motorBR, motorBL;
 
void writeAll(int us) {
    motorFL.writeMicroseconds(us);
    motorFR.writeMicroseconds(us);
    motorBR.writeMicroseconds(us);
    motorBL.writeMicroseconds(us);
}
 
void writeMotor(int motor, int us) {
    switch (motor) {
        case 1: motorFL.writeMicroseconds(us); break;
        case 2: motorFR.writeMicroseconds(us); break;
        case 3: motorBR.writeMicroseconds(us); break;
        case 4: motorBL.writeMicroseconds(us); break;
    }
}
 
void stopAll() {
    writeAll(CAL_MIN_US);
}
 
void printMenu() {
    Serial.println("\n========================================");
    Serial.println(" MOTOR TEST MENU");
    Serial.println("========================================");
    Serial.println(" c  — Run ESC calibration sequence");
    Serial.println(" 1  — Spin FL motor only (slow)");
    Serial.println(" 2  — Spin FR motor only (slow)");
    Serial.println(" 3  — Spin BR motor only (slow)");
    Serial.println(" 4  — Spin BL motor only (slow)");
    Serial.println(" a  — Spin ALL motors (slow)");
    Serial.println(" s  — STOP all motors");
    Serial.println(" +  — Increase test speed");
    Serial.println(" -  — Decrease test speed");
    Serial.println(" m  — Print this menu");
    Serial.println("========================================");
    Serial.println("!! PROPS OFF !!");
    Serial.println();
}
 
// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
 
    Serial.println("\n================================");
    Serial.println(" BLHeli ESC Calibration Tool");
    Serial.println("================================");
    Serial.println("!! REMOVE ALL PROPELLERS NOW !!");
    Serial.println();
 
    // Attach servos
    motorFL.attach(MOTOR_FL_PIN, CAL_MIN_US, FLIGHT_MAX);
    motorFR.attach(MOTOR_FR_PIN, CAL_MIN_US, FLIGHT_MAX);
    motorBR.attach(MOTOR_BR_PIN, CAL_MIN_US, FLIGHT_MAX);
    motorBL.attach(MOTOR_BL_PIN, CAL_MIN_US, FLIGHT_MAX);
 
    // Start at min — safe
    writeAll(CAL_MIN_US);
    delay(1000);
 
    Serial.println("Motors attached. Sending MIN signal.");
    Serial.println();
    Serial.println("Type 'c' to start ESC calibration");
    Serial.println("Type 'm' for motor test menu");
    Serial.println();
}
 
// ================================================================
// CALIBRATION SEQUENCE
// ================================================================
void runCalibration() {
    Serial.println("\n========================================");
    Serial.println(" ESC CALIBRATION SEQUENCE");
    Serial.println("========================================");
    Serial.println("Step 1: UNPLUG your battery / ESC power now.");
    Serial.println("        (Keep the ESP32 USB plugged in)");
    Serial.println("        Press ENTER when battery is disconnected...");
 
    // Wait for enter
    while (Serial.available()) Serial.read();
    while (Serial.read() != '\n') { delay(10); }
 
    Serial.println("\nStep 2: Sending MAX signal now...");
    writeAll(CAL_MAX_US);
    Serial.println("        MAX signal active (2000us)");
    Serial.println();
    Serial.println("Step 3: PLUG IN your battery now.");
    Serial.println("        You will hear: beep-beep (high-low tone)");
    Serial.println("        This means the ESC recognised the high point.");
    Serial.println("        Press ENTER after you hear the confirmation beeps...");
 
    while (Serial.available()) Serial.read();
    while (Serial.read() != '\n') { delay(10); }
 
    Serial.println("\nStep 4: Sending MIN signal now...");
    writeAll(CAL_MIN_US);
    Serial.println("        MIN signal active (1000us)");
    Serial.println();
    Serial.println("        You will hear: long beep (or melody)");
    Serial.println("        This means calibration is SAVED.");
    Serial.println();
    Serial.println("        Wait 3 seconds for ESCs to finish...");
    delay(3000);
 
    Serial.println("========================================");
    Serial.println(" CALIBRATION COMPLETE");
    Serial.println(" All ESCs now share the same range.");
    Serial.println(" Use motor test menu to verify.");
    Serial.println("========================================");
    printMenu();
}
 
// ================================================================
// LOOP
// ================================================================
int testSpeed = 1180;   // starting test speed — low and safe
int activeMotor = 0;    // 0 = none, 1-4 = individual, 5 = all
 
void loop() {
    if (Serial.available()) {
        char c = Serial.read();
 
        // Flush rest of line
        while (Serial.available() && Serial.peek() == '\n') Serial.read();
 
        switch (c) {
            case 'c':
            case 'C':
                stopAll();
                activeMotor = 0;
                runCalibration();
                break;
 
            case '1':
                stopAll();
                activeMotor = 1;
                motorFL.writeMicroseconds(testSpeed);
                Serial.printf("FL motor spinning at %dus\n", testSpeed);
                break;
 
            case '2':
                stopAll();
                activeMotor = 2;
                motorFR.writeMicroseconds(testSpeed);
                Serial.printf("FR motor spinning at %dus\n", testSpeed);
                break;
 
            case '3':
                stopAll();
                activeMotor = 3;
                motorBR.writeMicroseconds(testSpeed);
                Serial.printf("BR motor spinning at %dus\n", testSpeed);
                break;
 
            case '4':
                stopAll();
                activeMotor = 4;
                motorBL.writeMicroseconds(testSpeed);
                Serial.printf("BL motor spinning at %dus\n", testSpeed);
                break;
 
            case 'a':
            case 'A':
                activeMotor = 5;
                writeAll(testSpeed);
                Serial.printf("ALL motors spinning at %dus\n", testSpeed);
                break;
 
            case 's':
            case 'S':
                stopAll();
                activeMotor = 0;
                Serial.println("All motors stopped");
                break;
 
            case '+':
                testSpeed = min(testSpeed + 20, 1400);  // cap at 1400 for safety
                Serial.printf("Test speed: %dus\n", testSpeed);
                // Update active motor(s) immediately
                if      (activeMotor == 5) writeAll(testSpeed);
                else if (activeMotor > 0)  writeMotor(activeMotor, testSpeed);
                break;
 
            case '-':
                testSpeed = max(testSpeed - 20, FLIGHT_MIN);
                Serial.printf("Test speed: %dus\n", testSpeed);
                if      (activeMotor == 5) writeAll(testSpeed);
                else if (activeMotor > 0)  writeMotor(activeMotor, testSpeed);
                break;
 
            case 'm':
            case 'M':
                printMenu();
                break;
 
            case '\n':
            case '\r':
                break;
 
            default:
                Serial.printf("Unknown command '%c' — type 'm' for menu\n", c);
                break;
        }
    }
}
 