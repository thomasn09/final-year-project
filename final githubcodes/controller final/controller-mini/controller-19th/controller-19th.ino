/*
 * controller.ino — Handheld Controller ESP32
 * ─────────────────────────────────────────────────────────────────
 * Sends:    ControlPacket → Mini Drone C3 via ESP-NOW
 * Receives: DroneStatusPacket ← Pi Bridge ESP32 via ESP-NOW
 *
 * LCD behaviour:
 *   Normal:         Row0 = arm/hover/status   Row1 = throttle/send
 *   Drone seen:     Row0 = "DRONE SEEN!"       Row1 = "Land? [LAND btn]"
 *   Landing sent:   Row0 = "Landing..."        Row1 = "Sequence active"
 *
 * Buttons:
 *   ARM switch       — arms the mini drone
 *   TAKEOFF button   — triggers takeoff sequence
 *   LAND button      — triggers landing sequence (also used when drone seen)
 *   HOVER button     — holds = altitude hold mode active
 *   ESTOP switch     — immediate motor cut
 *   CALIB button     — recalibrate stick centres (only when disarmed)
 *
 * MAC addresses to configure:
 *   miniDroneMAC[]  — C3 MAC (printed at C3 boot)
 *   piBridgeMAC[]   — Pi Bridge MAC (printed at bridge boot)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================================================================
// CONFIG
// ================================================================
constexpr uint16_t LOOP_HZ                     = 50;
constexpr uint16_t LOOP_DT_MS                  = 1000 / LOOP_HZ;
constexpr int      THROTTLE_MIN_US             = 1100;
constexpr int      THROTTLE_MAX_US             = 1700;   // raised from 1400
constexpr int      THROTTLE_RISE_RATE_US_PER_S = 400;
constexpr int      THROTTLE_FALL_RATE_US_PER_S = 500;
constexpr float    STICK_SMOOTH_ALPHA          = 0.25f;
constexpr int      CENTER_DEADBAND             = 20;
constexpr uint8_t  ESPNOW_CHANNEL              = 1;
constexpr uint32_t STATUS_TIMEOUT_MS           = 1000;  // Pi bridge silence → droneFound=false

// ================================================================
// PINS
// ================================================================
const int JOY_LEFT_Y  = 39;   // Throttle
const int JOY_LEFT_X  = 36;   // Yaw
const int JOY_RIGHT_Y = 34;   // Pitch
const int JOY_RIGHT_X = 35;   // Roll

const int ARM_SWITCH_PIN     = 27;   // toggle switch — arm
const int ESTOP_SWITCH_PIN   = 33;   // toggle switch — emergency stop
const int TAKEOFF_BUTTON_PIN = 25;   // momentary button — takeoff
const int LAND_BUTTON_PIN    = 26;   // momentary button — land
const int HOVER_BUTTON_PIN   = 14;   // left joystick click — hold for altitude hold
const int CALIB_BUTTON_PIN   = 12;   // right joystick click — recalibrate sticks
// GPIO 16 & 17 unused — UART2 pins, too noisy for inputs

const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;

// ================================================================
// MACs  — UPDATE THESE
// ================================================================
// C3 prints its MAC when it boots — paste it here
uint8_t miniDroneMAC[] = {0x7C, 0x2C, 0x67, 0xC9, 0x08, 0x60};  // ← update

// Pi Bridge prints its MAC when it boots — paste it here
uint8_t piBridgeMAC[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // ← update

// ================================================================
// PACKETS
// ================================================================

// Sent to C3 — MUST match c3_flight_controller.ino exactly
#pragma pack(push, 1)
struct ControlPacket {
    int16_t throttle;       // 1000–2000 us
    int16_t yaw;            // −500 to +500
    int16_t pitch;          // −500 to +500
    int16_t roll;           // −500 to +500
    bool    hoverMode;      // true = altitude hold active
    bool    emergencyStop;
    bool    arm;            // true = drone should be armed
    bool    takeoff;        // edge trigger — start takeoff sequence
    bool    land;           // edge trigger — start landing sequence
};
#pragma pack(pop)

// Received from Pi Bridge — MUST match pi_bridge.ino exactly
#pragma pack(push, 1)
struct DroneStatusPacket {
    bool     droneSeen;
    float    tagX;        // -1..+1 from frame centre
    float    tagY;
    uint16_t tagSize;     // pixel size — bigger = closer
    int16_t  tagAngle;    // degrees
    uint8_t  tagId;
};
#pragma pack(pop)

ControlPacket    cmd;
DroneStatusPacket piStatus;

// ================================================================
// INTERRUPT FLAGS
// ================================================================
volatile bool isrEStop       = false;
volatile bool isrCalib       = false;
volatile bool isrLandEdge    = false;
volatile bool isrTakeoffEdge = false;

// ESTOP no longer uses interrupt — GPIO16 is UART2 RX and picks up noise
// It is now polled with a debounce in loop() instead
void IRAM_ATTR isrCalibHandler() {
    static uint32_t last = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last > 300) { isrCalib = true; last = now; }
}
void IRAM_ATTR isrLandHandler() {
    static uint32_t last = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last > 300) { isrLandEdge = true; last = now; }
}
void IRAM_ATTR isrTakeoffHandler() {
    static uint32_t last = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last > 300) { isrTakeoffEdge = true; last = now; }
}

// ================================================================
// STATE
// ================================================================
bool          droneFound       = false;
unsigned long lastStatusMs     = 0;
bool          lastSendOK       = false;
bool          landingActive    = false;   // true once landing command sent
unsigned long landingSentMs    = 0;

float yawFilt = 0, pitchFilt = 0, rollFilt = 0;
int   throttleOut = THROTTLE_MIN_US;
int   centerThr = 0, centerYaw = 0, centerPitch = 0, centerRoll = 0;

// ================================================================
// HELPERS
// ================================================================
template<typename T>
T clamp(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

int readADC_avg(int pin, int samples = 8) {
    long s = 0;
    for (int i = 0; i < samples; i++) { s += analogRead(pin); delay(2); }
    return (int)(s / samples);
}

float smooth(float o, float m, float a) { return o + a * (m - o); }

int16_t mapCenteredRaw(int raw, int center) {
    float val = ((float)(raw - center) / 2048.0f) * 500.0f;  // ±500 range
    int   out = (int)roundf(val);
    if (abs(out) < CENTER_DEADBAND) out = 0;
    return clamp<int>(out, -500, 500);
}

int desiredThrottleFromRaw(int raw) {
    const int DZ = 60;
    if (abs(raw - centerThr) <= DZ) return THROTTLE_MIN_US;
    float frac = clamp<float>((float)(raw - centerThr) / (4095 - centerThr), 0, 1);
    return clamp<int>(THROTTLE_MIN_US + (int)roundf(frac * (THROTTLE_MAX_US - THROTTLE_MIN_US)),
                      THROTTLE_MIN_US, THROTTLE_MAX_US);
}

int rateLimitThrottle(int desired, int current, float dt_s) {
    int maxRise = (int)(THROTTLE_RISE_RATE_US_PER_S * dt_s);
    int maxFall = (int)(THROTTLE_FALL_RATE_US_PER_S * dt_s);
    int delta   = desired - current;
    if (delta > 0) delta = min(delta, maxRise);
    else           delta = max(delta, -maxFall);
    return clamp<int>(current + delta, THROTTLE_MIN_US, THROTTLE_MAX_US);
}

// ================================================================
// CALIBRATION
// ================================================================
void calibrateSticks() {
    Serial.println("Calibrating — leave sticks centred");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Calibrating...");
    lcd.setCursor(0, 1); lcd.print("Leave centred");
    delay(500);
    centerThr   = readADC_avg(JOY_LEFT_Y,  20);
    centerYaw   = readADC_avg(JOY_LEFT_X,  20);
    centerPitch = readADC_avg(JOY_RIGHT_Y, 20);
    centerRoll  = readADC_avg(JOY_RIGHT_X, 20);
    Serial.printf("Centers T:%d Y:%d P:%d R:%d\n",
                  centerThr, centerYaw, centerPitch, centerRoll);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Cal done");
    delay(700);
}

// ================================================================
// LCD
// ================================================================
void updateLCD(bool armed, bool hover) {
    // Priority display: landing active > drone seen > normal
    if (landingActive && (millis() - landingSentMs < 5000)) {
        lcd.setCursor(0, 0); lcd.print("Landing...      ");
        lcd.setCursor(0, 1); lcd.print("Sequence active ");
        return;
    }

    if (droneFound) {
        lcd.setCursor(0, 0); lcd.print("DRONE SEEN!     ");
        lcd.setCursor(0, 1); lcd.print("Land? [LAND btn]");
        return;
    }

    // Normal display
    lcd.setCursor(0, 0);
    char row0[17];
    snprintf(row0, sizeof(row0), "%s %s %s",
             armed  ? "ARM" : "dis",
             hover  ? "HOV" : "   ",
             lastSendOK ? "OK " : "ERR");
    lcd.print(row0);

    lcd.setCursor(0, 1);
    char row1[17];
    snprintf(row1, sizeof(row1), "T:%-4d Tag:%s",
             throttleOut,
             droneFound ? "YES" : " NO");
    lcd.print(row1);
}

// ================================================================
// ESP-NOW CALLBACKS
// ================================================================
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    // Accept DroneStatusPacket from Pi Bridge
    if (len == sizeof(DroneStatusPacket)) {
        memcpy(&piStatus, data, len);
        droneFound   = piStatus.droneSeen;
        lastStatusMs = millis();

        if (droneFound) {
            Serial.printf("Pi: TAG SEEN  ID:%d  X:%.2f  Y:%.2f  Size:%d  Angle:%d\n",
                          piStatus.tagId, piStatus.tagX, piStatus.tagY,
                          piStatus.tagSize, piStatus.tagAngle);
        }
    }
}

void onSent(const esp_now_send_info_t* info, esp_now_send_status_t status) {
    lastSendOK = (status == ESP_NOW_SEND_SUCCESS);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(ARM_SWITCH_PIN,     INPUT_PULLUP);
    pinMode(ESTOP_SWITCH_PIN,   INPUT_PULLUP);
    pinMode(CALIB_BUTTON_PIN,   INPUT_PULLUP);
    pinMode(HOVER_BUTTON_PIN,   INPUT_PULLUP);
    pinMode(LAND_BUTTON_PIN,    INPUT_PULLUP);
    pinMode(TAKEOFF_BUTTON_PIN, INPUT_PULLUP);

    // ESTOP is polled, not interrupt-driven — avoids UART noise on GPIO16
    attachInterrupt(digitalPinToInterrupt(CALIB_BUTTON_PIN),   isrCalibHandler,   FALLING);
    attachInterrupt(digitalPinToInterrupt(LAND_BUTTON_PIN),    isrLandHandler,    FALLING);
    attachInterrupt(digitalPinToInterrupt(TAKEOFF_BUTTON_PIN), isrTakeoffHandler, FALLING);

    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Controller Boot");

    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("Controller MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println(">>> Paste this MAC into pi_bridge.ino as controllerMAC[] <<<");

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("MAC:");
    lcd.setCursor(0, 1); lcd.print(WiFi.macAddress().substring(0, 16));
    delay(2000);

    calibrateSticks();

    if (esp_now_init() != ESP_OK) {
        lcd.clear(); lcd.print("ESP-NOW FAIL");
        while (true) delay(10);
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    // Register mini drone C3 as peer
    esp_now_peer_info_t peer1 = {};
    memcpy(peer1.peer_addr, miniDroneMAC, 6);
    peer1.channel = ESPNOW_CHANNEL;
    peer1.encrypt = false;
    if (esp_now_add_peer(&peer1) != ESP_OK) {
        lcd.clear(); lcd.print("MINI PEER FAIL");
        while (true) delay(10);
    }

    // Register Pi Bridge as peer (to receive from it)
    esp_now_peer_info_t peer2 = {};
    memcpy(peer2.peer_addr, piBridgeMAC, 6);
    peer2.channel = ESPNOW_CHANNEL;
    peer2.encrypt = false;
    esp_now_add_peer(&peer2);   // non-fatal if bridge MAC not yet set

    lastStatusMs = millis();
    Serial.println("\nController ready");
    Serial.println("Switches: ARM=GPIO25  ESTOP=GPIO16  HOVER=GPIO27");
    Serial.println("Buttons:  TAKEOFF=GPIO13  LAND=GPIO33  CALIB=GPIO32");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Ready");
    lcd.setCursor(0, 1); lcd.print("Arm to fly");
    delay(1000);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    float dt_s = LOOP_DT_MS / 1000.0f;

    // ── Status timeout: no packet from Pi Bridge for 1 s → not seen ──
    if (millis() - lastStatusMs > STATUS_TIMEOUT_MS) {
        droneFound = false;
    }

    // ── ESTOP — debounced poll (not interrupt, GPIO16 is noisy) ──
    {
        static bool     lastEStopState = false;
        static uint32_t eStopStableMs  = 0;
        bool rawEStop = !digitalRead(ESTOP_SWITCH_PIN);   // LOW = pressed

        if (rawEStop != lastEStopState) {
            eStopStableMs  = millis();
            lastEStopState = rawEStop;
        }
        // Only act if pin has been consistently LOW for 50 ms
        if (rawEStop && (millis() - eStopStableMs > 50)) {
            isrEStop = true;
        }
    }
    if (isrEStop) {
        isrEStop = false;
        throttleOut = THROTTLE_MIN_US;
        rollFilt = pitchFilt = yawFilt = 0.0f;
        landingActive = false;
        memset(&cmd, 0, sizeof(cmd));
        cmd.throttle      = THROTTLE_MIN_US;
        cmd.emergencyStop = true;
        esp_now_send(miniDroneMAC, (uint8_t*) &cmd, sizeof(cmd));
        lcd.clear(); lcd.setCursor(0, 0); lcd.print("!!! ESTOP !!!");
        Serial.println("!!! ESTOP FIRED !!!");
    }

    // ── Calibration (only when disarmed) ──
    if (isrCalib) {
        isrCalib = false;
        if (digitalRead(ARM_SWITCH_PIN)) {  // HIGH = disarmed (pull-up)
            calibrateSticks();
            rollFilt = pitchFilt = yawFilt = 0.0f;
            throttleOut = THROTTLE_MIN_US;
        }
    }

    // ── Read switches ──
    bool armed      = !digitalRead(ARM_SWITCH_PIN);
    bool hoverHeld  = !digitalRead(HOVER_BUTTON_PIN);
    bool eStopHeld  = !digitalRead(ESTOP_SWITCH_PIN);

    // ── Takeoff edge ──
    bool sendTakeoff = false;
    if (isrTakeoffEdge) {
        isrTakeoffEdge = false;
        sendTakeoff    = true;
        landingActive  = false;
        Serial.println(">>> Takeoff command");
    }

    // ── Land edge — also fires when drone seen + land pressed ──
    bool sendLand = false;
    if (isrLandEdge) {
        isrLandEdge   = false;
        sendLand      = true;
        landingActive = true;
        landingSentMs = millis();
        if (droneFound) {
            Serial.printf(">>> Land command (tag ID:%d  X:%.2f  Y:%.2f  Size:%d  Angle:%d)\n",
                          piStatus.tagId, piStatus.tagX, piStatus.tagY,
                          piStatus.tagSize, piStatus.tagAngle);
        } else {
            Serial.println(">>> Land command");
        }
    }

    // ── Read sticks ──
    int rawThr = readADC_avg(JOY_LEFT_Y,  4);
    int rawYaw = readADC_avg(JOY_LEFT_X,  4);
    int rawPit = readADC_avg(JOY_RIGHT_Y, 4);
    int rawRol = readADC_avg(JOY_RIGHT_X, 4);

    yawFilt   = smooth(yawFilt,   mapCenteredRaw(rawYaw, centerYaw),   STICK_SMOOTH_ALPHA);
    pitchFilt = smooth(pitchFilt, mapCenteredRaw(rawPit, centerPitch), STICK_SMOOTH_ALPHA);
    rollFilt  = smooth(rollFilt,  mapCenteredRaw(rawRol, centerRoll),  STICK_SMOOTH_ALPHA);

    // ── Throttle ──
    int desired = armed ? desiredThrottleFromRaw(rawThr) : THROTTLE_MIN_US;
    throttleOut = rateLimitThrottle(desired, throttleOut, dt_s);
    if (eStopHeld) {
        throttleOut = THROTTLE_MIN_US;
        rollFilt = pitchFilt = yawFilt = 0.0f;
    }

    // ── Build command packet ──
    cmd.throttle      = (int16_t) throttleOut;
    cmd.yaw           = (int16_t) roundf(yawFilt);
    cmd.pitch         = (int16_t) roundf(pitchFilt);
    cmd.roll          = (int16_t) roundf(rollFilt);
    cmd.hoverMode     = hoverHeld;
    cmd.emergencyStop = eStopHeld;
    cmd.arm           = armed;
    cmd.takeoff       = sendTakeoff;
    cmd.land          = sendLand;

    // ── Send to mini drone C3 ──
    esp_now_send(miniDroneMAC, (uint8_t*) &cmd, sizeof(cmd));

    // Clear edge triggers after send
    cmd.takeoff = false;
    cmd.land    = false;

    // ── Serial debug ──
    Serial.printf(
        "ARM:%d HOV:%d | T:%d Y:%d P:%d R:%d | SEEN:%s ID:%d X:%.2f Y:%.2f Sz:%d | S:%s\n",
        armed, hoverHeld,
        throttleOut,
        (int)roundf(yawFilt), (int)roundf(pitchFilt), (int)roundf(rollFilt),
        droneFound ? "YES" : "NO", piStatus.tagId, piStatus.tagX, piStatus.tagY, piStatus.tagSize,
        lastSendOK ? "OK" : "FAIL");

    // ── LCD ──
    updateLCD(armed, hoverHeld);

    delay(LOOP_DT_MS);
}