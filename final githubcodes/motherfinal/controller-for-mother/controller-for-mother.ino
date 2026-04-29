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
constexpr uint8_t  ESPNOW_CHANNEL              = 1;
constexpr uint16_t LOOP_HZ                     = 50;
constexpr uint16_t LOOP_DT_MS                  = 1000 / LOOP_HZ;
 
// Throttle range sent to drone (microseconds)
// Keep MAX low for first flights — raise once you're comfortable
constexpr int THROTTLE_MIN_US  = 1100;   // disarmed / min
constexpr int THROTTLE_MAX_US  = 1600;   // max for first flights (safe ceiling)
constexpr int THROTTLE_LOCK_US = 1200;   // throttle lock test value
 
// How fast throttle can ramp up/down (us per second)
constexpr int THROTTLE_RISE_RATE = 300;
constexpr int THROTTLE_FALL_RATE = 600;  // falls faster for safety
 
// Stick smoothing
constexpr float STICK_ALPHA   = 0.20f;   // lower = smoother but slower response
constexpr int   STICK_DEADBAND = 15;     // stick dead zone (out of 100)
 
// ================================================================
// PINS
// ================================================================
const int JOY_LEFT_Y        = 39;   // Throttle
const int JOY_LEFT_X        = 36;   // Yaw
const int JOY_RIGHT_Y       = 34;   // Pitch
const int JOY_RIGHT_X       = 35;   // Roll
const int ARM_SWITCH_PIN    = 25;
const int THROTTLE_LOCK_PIN = 26;
const int ESTOP_PIN         = 16;
const int CALIB_PIN         = 32;
const int HOVER_PIN         = 27;
const int LCD_SDA           = 21;
const int LCD_SCL           = 22;
 
// ================================================================
// MAC — motherdrone
// ================================================================
uint8_t motherdroneMAC[] = {0x44, 0x1D, 0x64, 0xF7, 0x11, 0xC0};
 
// ================================================================
// PACKET — must match ControlPacket on the drone exactly
// ================================================================
typedef struct {
    int16_t throttle;      // 1100–2000 us
    int16_t yaw;           // -100 to 100
    int16_t pitch;         // -100 to 100
    int16_t roll;          // -100 to 100
    bool    hoverMode;
    bool    emergencyStop;
} ControlPacket;
 
ControlPacket cmd;
 
// ================================================================
// INTERRUPT FLAGS
// ================================================================
volatile bool isrEStop = false;
volatile bool isrCalib = false;
 
void IRAM_ATTR isrEstopHandler() {
    isrEStop = true;
}
 
void IRAM_ATTR isrCalibHandler() {
    static uint32_t lastMs = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - lastMs > 300) { isrCalib = true; lastMs = now; }
}
 
// ================================================================
// STATE
// ================================================================
bool  lastSendOK   = false;
float yawFilt      = 0, pitchFilt = 0, rollFilt = 0;
int   throttleOut  = THROTTLE_MIN_US;
int   centerThr    = 2048, centerYaw = 2048;
int   centerPitch  = 2048, centerRoll = 2048;
 
// ================================================================
// HELPERS
// ================================================================
template<typename T>
T clamp(T v, T lo, T hi) { return v < lo ? lo : v > hi ? hi : v; }
 
int readADC(int pin, int n = 6) {
    long s = 0;
    for (int i = 0; i < n; i++) { s += analogRead(pin); delay(1); }
    return (int)(s / n);
}
 
float smooth(float old, float in, float a) { return old + a * (in - old); }
 
// Maps raw ADC to -100..100 with deadband
int16_t mapStick(int raw, int center) {
    float v = ((float)(raw - center) / 2048.0f) * 100.0f;
    int out = (int)roundf(v);
    if (abs(out) < STICK_DEADBAND) out = 0;
    return (int16_t)clamp(out, -100, 100);
}
 
// Maps throttle raw ADC to THROTTLE_MIN_US..THROTTLE_MAX_US
int mapThrottle(int raw) {
    // Dead zone at bottom — stick must be clearly pushed up to move
    const int DZ = 80;
    if (raw <= centerThr + DZ) return THROTTLE_MIN_US;
    float frac = clamp<float>((float)(raw - centerThr - DZ) / (float)(4095 - centerThr - DZ), 0.0f, 1.0f);
    return clamp<int>(THROTTLE_MIN_US + (int)(frac * (THROTTLE_MAX_US - THROTTLE_MIN_US)),
                      THROTTLE_MIN_US, THROTTLE_MAX_US);
}
 
// Rate-limits throttle so it can't jump instantly
int rateLimit(int desired, int current, float dt) {
    int maxUp   = (int)(THROTTLE_RISE_RATE * dt);
    int maxDown = (int)(THROTTLE_FALL_RATE * dt);
    int d = desired - current;
    if (d > 0) d = min(d, maxUp);
    else       d = max(d, -maxDown);
    return clamp<int>(current + d, THROTTLE_MIN_US, THROTTLE_MAX_US);
}
 
// ================================================================
// CALIBRATION
// ================================================================
void calibrateSticks() {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Calibrating...");
    lcd.setCursor(0, 1); lcd.print("Centre sticks");
    Serial.println("Calibrating — leave sticks centred");
    delay(600);
 
    centerThr   = readADC(JOY_LEFT_Y,  30);
    centerYaw   = readADC(JOY_LEFT_X,  30);
    centerPitch = readADC(JOY_RIGHT_Y, 30);
    centerRoll  = readADC(JOY_RIGHT_X, 30);
 
    Serial.printf("Cal done: T=%d Y=%d P=%d R=%d\n",
        centerThr, centerYaw, centerPitch, centerRoll);
 
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Cal done!");
    delay(800);
}
 
// ================================================================
// LCD UPDATE
// Row 0: ARM state | Send status
// Row 1: Throttle  | Hover
// ================================================================
void updateLCD(bool armed, bool hover) {
    lcd.setCursor(0, 0);
    lcd.print(armed ? "ARMED  " : "DISARM ");
    lcd.print(lastSendOK ? "TX:OK " : "TX:FAIL");
 
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(throttleOut);
    lcd.print(hover ? " HOV" : "    ");
}
 
// ================================================================
// ESP-NOW CALLBACKS
// ================================================================
void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    lastSendOK = (status == ESP_NOW_SEND_SUCCESS);
}
 
// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== BIG DRONE CONTROLLER BOOT ===");
 
    pinMode(ARM_SWITCH_PIN,    INPUT_PULLUP);
    pinMode(THROTTLE_LOCK_PIN, INPUT_PULLUP);
    pinMode(ESTOP_PIN,         INPUT_PULLUP);
    pinMode(CALIB_PIN,         INPUT_PULLUP);
    pinMode(HOVER_PIN,         INPUT_PULLUP);
 
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN),  isrEstopHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(CALIB_PIN),  isrCalibHandler, FALLING);
 
    Wire.begin(LCD_SDA, LCD_SCL);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Big Drone Ctrl");
    lcd.setCursor(0, 1); lcd.print("Booting...");
 
    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
 
    Serial.print("Controller MAC: ");
    Serial.println(WiFi.macAddress());
 
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW FAILED");
        lcd.clear(); lcd.print("ESP-NOW FAILED");
        while (true) delay(10);
    }
 
    esp_now_register_send_cb(onSent);
 
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, motherdroneMAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Drone peer FAILED");
        lcd.clear(); lcd.print("PEER FAILED");
        while (true) delay(10);
    }
 
    Serial.println("Drone peer added");
 
    calibrateSticks();
 
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Ready!");
    lcd.setCursor(0, 1); lcd.print("Flip ARM to arm");
    delay(1000);
    Serial.println("Controller ready — flip ARM switch to arm");
}
 
// ================================================================
// LOOP
// ================================================================
void loop() {
    float dt = LOOP_DT_MS / 1000.0f;
 
    // ---- ESTOP — highest priority ----
    if (isrEStop) {
        isrEStop              = false;
        throttleOut           = THROTTLE_MIN_US;
        yawFilt = pitchFilt = rollFilt = 0.0f;
        cmd.throttle      = THROTTLE_MIN_US;
        cmd.yaw           = 0;
        cmd.pitch         = 0;
        cmd.roll          = 0;
        cmd.hoverMode     = false;
        cmd.emergencyStop = true;
        esp_now_send(motherdroneMAC, (uint8_t*)&cmd, sizeof(cmd));
        Serial.println("!!! ESTOP !!!");
 
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("!!! E-STOP !!!");
        lcd.setCursor(0, 1); lcd.print("Disarm to reset");
        delay(LOOP_DT_MS);
        return;
    }
 
    // ---- Calibration ----
    if (isrCalib) {
        isrCalib = false;
        bool armed = !digitalRead(ARM_SWITCH_PIN);
        if (!armed) {
            calibrateSticks();
            throttleOut = THROTTLE_MIN_US;
            yawFilt = pitchFilt = rollFilt = 0.0f;
        } else {
            Serial.println("Disarm first to calibrate");
        }
    }
 
    // ---- Read switches ----
    bool armed          = !digitalRead(ARM_SWITCH_PIN);
    bool throttleLocked = !digitalRead(THROTTLE_LOCK_PIN);
    bool hoverMode      = !digitalRead(HOVER_PIN);
    bool eStopHeld      = !digitalRead(ESTOP_PIN);
 
    // ---- Read sticks ----
    int rawThr = readADC(JOY_LEFT_Y,  4);
    int rawYaw = readADC(JOY_LEFT_X,  4);
    int rawPit = readADC(JOY_RIGHT_Y, 4);
    int rawRol = readADC(JOY_RIGHT_X, 4);
 
    // Smooth yaw/pitch/roll
    yawFilt   = smooth(yawFilt,   (float)mapStick(rawYaw, centerYaw),   STICK_ALPHA);
    pitchFilt = smooth(pitchFilt, (float)mapStick(rawPit, centerPitch), STICK_ALPHA);
    rollFilt  = smooth(rollFilt,  (float)mapStick(rawRol, centerRoll),  STICK_ALPHA);
 
    // ---- Throttle ----
    if (!armed || eStopHeld) {
        // Not armed — ramp throttle back to min quickly
        throttleOut = rateLimit(THROTTLE_MIN_US, throttleOut, dt);
    } else if (throttleLocked) {
        throttleOut = rateLimit(THROTTLE_LOCK_US, throttleOut, dt);
    } else {
        int desired = mapThrottle(rawThr);
        throttleOut = rateLimit(desired, throttleOut, dt);
    }
 
    // ---- Build packet ----
    cmd.throttle      = (int16_t)throttleOut;
    cmd.yaw           = (int16_t)roundf(yawFilt);
    cmd.pitch         = (int16_t)roundf(pitchFilt);
    cmd.roll          = (int16_t)roundf(rollFilt);
    cmd.hoverMode     = hoverMode;
    cmd.emergencyStop = eStopHeld;
 
    // ---- Send ----
    esp_now_send(motherdroneMAC, (uint8_t*)&cmd, sizeof(cmd));
 
    // ---- Serial status ----
    Serial.printf("ARM:%d LOCK:%d HOV:%d | T:%d Y:%d P:%d R:%d | TX:%s\n",
        armed, throttleLocked, hoverMode,
        throttleOut,
        (int)roundf(yawFilt),
        (int)roundf(pitchFilt),
        (int)roundf(rollFilt),
        lastSendOK ? "OK" : "FAIL"
    );
 
    updateLCD(armed, hoverMode);
    delay(LOOP_DT_MS);
}