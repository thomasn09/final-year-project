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
constexpr int      THROTTLE_MAX_US             = 1400;
constexpr int      TEST_THROTTLE_LOCK_US       = 1200;
constexpr int      THROTTLE_RISE_RATE_US_PER_S = 400;
constexpr int      THROTTLE_FALL_RATE_US_PER_S = 500;
constexpr float    STICK_SMOOTH_ALPHA          = 0.25f;
constexpr int      CENTER_DEADBAND             = 20;
constexpr uint8_t  ESPNOW_CHANNEL              = 1;

// ================================================================
// PINS
// ================================================================
// Sticks
const int JOY_LEFT_Y  = 39;   // Throttle
const int JOY_LEFT_X  = 36;   // Yaw   ← left stick X
const int JOY_RIGHT_Y = 34;   // Pitch
const int JOY_RIGHT_X = 35;   // Roll  ← right stick X

// Motherdrone buttons
const int ARM_SWITCH_PIN    = 25;
const int THROTTLE_LOCK_PIN = 26;
const int ESTOP_SWITCH_PIN  = 16;
const int CALIB_BUTTON_PIN  = 32;
const int HOVER_BUTTON_PIN  = 27;   // new — hover mode for motherdrone

// Mini drone buttons
const int LAND_BUTTON_PIN       = 33;
const int TAKEOFF_BUTTON_PIN    = 13;
const int MINI_ARM_SWITCH_PIN   = 14;

// LCD
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;

// ================================================================
// MACs
// ================================================================
uint8_t motherdroneMAC[] = {0x44, 0x1D, 0x64, 0xF7, 0x11, 0xC0};
uint8_t miniDroneMAC[]   = {0x98, 0x3D, 0xAE, 0x60, 0x1D, 0x50};

// ================================================================
// PACKETS
// ================================================================

// Sent to motherdrone — original format
typedef struct {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    bool    hoverMode;
    bool    emergencyStop;
} MotherPacket;

// Sent to mini drone — matches mini drone flight controller struct
typedef struct {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    bool    arm;
    bool    takeoff;
    bool    land;
    bool    emergencyStop;
} MiniPacket;

// Received from motherdrone
typedef struct {
    bool droneSeen;
} DroneStatusPacket;

MotherPacket motherCmd;
MiniPacket   miniCmd;

// ================================================================
// INTERRUPT FLAGS
// volatile — written by ISR, read in main loop
// IRAM_ATTR — keeps ISR in fast internal RAM so it always executes
// ================================================================
volatile bool isrEStop       = false;
volatile bool isrCalib       = false;
volatile bool isrLandEdge    = false;
volatile bool isrTakeoffEdge = false;

void IRAM_ATTR isrEstopHandler() {
    isrEStop = true;
}

void IRAM_ATTR isrCalibHandler() {
    // Simple debounce — ignore if triggered within 300ms of last call
    static uint32_t lastUs = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - lastUs > 300) {
        isrCalib = true;
        lastUs   = now;
    }
}

void IRAM_ATTR isrLandHandler() {
    static uint32_t lastUs = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - lastUs > 300) {
        isrLandEdge = true;
        lastUs      = now;
    }
}

void IRAM_ATTR isrTakeoffHandler() {
    static uint32_t lastUs = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - lastUs > 300) {
        isrTakeoffEdge = true;
        lastUs         = now;
    }
}

// ================================================================
// STATE
// ================================================================
bool          droneFound       = false;
bool          lastSendOK       = false;
unsigned long lastSerialPrompt = 0;

float yawFilt   = 0, pitchFilt = 0, rollFilt = 0;
int   throttleOut = THROTTLE_MIN_US;
int   centerThr=0, centerYaw=0, centerPitch=0, centerRoll=0;

// ================================================================
// HELPERS
// ================================================================
template<typename T>
T clamp(T v, T lo, T hi) { return (v<lo)?lo:(v>hi?hi:v); }

int readADC_avg(int pin, int samples=8) {
    long s=0;
    for(int i=0;i<samples;i++){s+=analogRead(pin);delay(2);}
    return (int)(s/samples);
}

float smooth(float o, float m, float a) { return o+a*(m-o); }

int16_t mapCenteredRaw(int raw, int center) {
    float val = ((float)(raw-center)/2048.0f)*100.0f;
    int out   = (int)roundf(val);
    if (abs(out)<CENTER_DEADBAND) out=0;
    return clamp<int>(out,-100,100);
}

int desiredThrottleFromRaw(int raw) {
    const int DZ=60;
    if (abs(raw-centerThr)<=DZ) return THROTTLE_MIN_US;
    float frac=clamp<float>((float)(raw-centerThr)/(4095-centerThr),0,1);
    return clamp<int>(THROTTLE_MIN_US+(int)roundf(frac*(THROTTLE_MAX_US-THROTTLE_MIN_US)),
                      THROTTLE_MIN_US,THROTTLE_MAX_US);
}

int rateLimitThrottle(int desired, int current, float dt_s) {
    int maxRise=(int)(THROTTLE_RISE_RATE_US_PER_S*dt_s);
    int maxFall=(int)(THROTTLE_FALL_RATE_US_PER_S*dt_s);
    int delta=desired-current;
    if(delta>0) delta=min(delta,maxRise);
    else        delta=max(delta,-maxFall);
    return clamp<int>(current+delta,THROTTLE_MIN_US,THROTTLE_MAX_US);
}

// ================================================================
// CALIBRATION
// ================================================================
void calibrateSticks() {
    Serial.println("Calibrating — leave sticks centred");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Calibrating...");
    lcd.setCursor(0,1); lcd.print("Leave centred");
    delay(500);

    centerThr   = readADC_avg(JOY_LEFT_Y,  20);
    centerYaw   = readADC_avg(JOY_LEFT_X,  20);   // Yaw on left X
    centerPitch = readADC_avg(JOY_RIGHT_Y, 20);
    centerRoll  = readADC_avg(JOY_RIGHT_X, 20);   // Roll on right X

    Serial.printf("Centers T:%d Y:%d P:%d R:%d\n",
        centerThr, centerYaw, centerPitch, centerRoll);
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Cal done");
    delay(700);
}

// ================================================================
// LCD
// Row 0: Mother ARM/DIS | Mini arm | Drone seen
// Row 1: Throttle | Send status
// ================================================================
void updateLCD(bool motherArmed, bool miniArmed) {
    lcd.setCursor(0,0);
    lcd.print(motherArmed ? "M:ARM " : "M:DIS ");
    lcd.print(miniArmed   ? "m:ARM " : "m:dis ");
    lcd.print(droneFound  ? "SEE" : "---");

    lcd.setCursor(0,1);
    lcd.print("T:"); lcd.print(throttleOut);
    lcd.print(" S:"); lcd.print(lastSendOK ? "OK  " : "FAIL");
}

// ================================================================
// ESP-NOW CALLBACKS
// ================================================================
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (memcmp(info->src_addr, motherdroneMAC, 6) == 0) {
        DroneStatusPacket status;
        memcpy(&status, data, sizeof(status));
        droneFound = status.droneSeen;
    }
}

void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    lastSendOK = (status == ESP_NOW_SEND_SUCCESS);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // Pin modes
    pinMode(ARM_SWITCH_PIN,      INPUT_PULLUP);
    pinMode(THROTTLE_LOCK_PIN,   INPUT_PULLUP);
    pinMode(ESTOP_SWITCH_PIN,    INPUT_PULLUP);
    pinMode(CALIB_BUTTON_PIN,    INPUT_PULLUP);
    pinMode(HOVER_BUTTON_PIN,    INPUT_PULLUP);
    pinMode(LAND_BUTTON_PIN,     INPUT_PULLUP);
    pinMode(TAKEOFF_BUTTON_PIN,  INPUT_PULLUP);
    pinMode(MINI_ARM_SWITCH_PIN, INPUT_PULLUP);

    // Hardware interrupts
    // ESTOP fires instantly on press — highest urgency
    // Buttons fire on FALLING (press down), debounced in ISR
    attachInterrupt(digitalPinToInterrupt(ESTOP_SWITCH_PIN),   isrEstopHandler,   FALLING);
    attachInterrupt(digitalPinToInterrupt(CALIB_BUTTON_PIN),   isrCalibHandler,   FALLING);
    attachInterrupt(digitalPinToInterrupt(LAND_BUTTON_PIN),    isrLandHandler,    FALLING);
    attachInterrupt(digitalPinToInterrupt(TAKEOFF_BUTTON_PIN), isrTakeoffHandler, FALLING);

    // LCD
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Controller Boot");
    lcd.setCursor(0,1); lcd.print("Starting...");

    // WiFi
    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.print("Controller MAC: ");
    Serial.println(WiFi.macAddress());

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("MAC Ready");
    lcd.setCursor(0,1); lcd.print(WiFi.macAddress().substring(0,16));
    delay(1000);

    calibrateSticks();

    if (esp_now_init() != ESP_OK) {
        lcd.clear(); lcd.print("ESP-NOW FAIL");
        while(true) delay(10);
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    // Add motherdrone as peer
    esp_now_peer_info_t peer1 = {};
    memcpy(peer1.peer_addr, motherdroneMAC, 6);
    peer1.channel = ESPNOW_CHANNEL;
    peer1.encrypt = false;
    if (esp_now_add_peer(&peer1) != ESP_OK) {
        lcd.clear(); lcd.print("MOTHER FAIL");
        while(true) delay(10);
    }

    // Add mini drone as peer
    esp_now_peer_info_t peer2 = {};
    memcpy(peer2.peer_addr, miniDroneMAC, 6);
    peer2.channel = ESPNOW_CHANNEL;
    peer2.encrypt = false;
    if (esp_now_add_peer(&peer2) != ESP_OK) {
        lcd.clear(); lcd.print("MINI FAIL");
        while(true) delay(10);
    }

    Serial.println("Controller Ready");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Controller Ready");
    lcd.setCursor(0,1); lcd.print("Waiting...");
    delay(1000);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    float dt_s = LOOP_DT_MS / 1000.0f;

    // ---- ESTOP interrupt — handled first, highest priority ----
    if (isrEStop) {
        isrEStop        = false;
        throttleOut     = THROTTLE_MIN_US;
        rollFilt = pitchFilt = yawFilt = 0.0f;
        motherCmd.emergencyStop = true;
        miniCmd.emergencyStop   = true;
        esp_now_send(motherdroneMAC, (uint8_t*)&motherCmd, sizeof(motherCmd));
        esp_now_send(miniDroneMAC,   (uint8_t*)&miniCmd,   sizeof(miniCmd));
        Serial.println("!!! ESTOP FIRED !!!");
    }

    // ---- Calibration interrupt ----
    if (isrCalib) {
        isrCalib = false;
        if (!digitalRead(ARM_SWITCH_PIN)) {   // only calibrate when disarmed
            calibrateSticks();
            rollFilt = pitchFilt = yawFilt = 0.0f;
            throttleOut = THROTTLE_MIN_US;
        }
    }

    // ---- Read switches (polled — ARM and HOVER are hold switches) ----
    bool motherArmed    = !digitalRead(ARM_SWITCH_PIN);
    bool throttleLocked = !digitalRead(THROTTLE_LOCK_PIN);
    bool hoverPressed   = !digitalRead(HOVER_BUTTON_PIN);
    bool miniArmed      = !digitalRead(MINI_ARM_SWITCH_PIN);
    bool eStopHeld      = !digitalRead(ESTOP_SWITCH_PIN);  // poll as backup

    // ---- Read sticks ----
    int rawThr = readADC_avg(JOY_LEFT_Y,  4);
    int rawYaw = readADC_avg(JOY_LEFT_X,  4);   // Yaw — left X
    int rawPit = readADC_avg(JOY_RIGHT_Y, 4);
    int rawRol = readADC_avg(JOY_RIGHT_X, 4);   // Roll — right X

    yawFilt   = smooth(yawFilt,   mapCenteredRaw(rawYaw, centerYaw),   STICK_SMOOTH_ALPHA);
    pitchFilt = smooth(pitchFilt, mapCenteredRaw(rawPit, centerPitch), STICK_SMOOTH_ALPHA);
    rollFilt  = smooth(rollFilt,  mapCenteredRaw(rawRol, centerRoll),  STICK_SMOOTH_ALPHA);

    // ---- Throttle ----
    int desiredThrottle = motherArmed ? desiredThrottleFromRaw(rawThr) : THROTTLE_MIN_US;
    if (throttleLocked && motherArmed) desiredThrottle = TEST_THROTTLE_LOCK_US;
    throttleOut = rateLimitThrottle(desiredThrottle, throttleOut, dt_s);
    if (eStopHeld) { throttleOut = THROTTLE_MIN_US; rollFilt = pitchFilt = yawFilt = 0.0f; }

    // ---- Build motherdrone packet ----
    motherCmd.throttle      = throttleOut;
    motherCmd.yaw           = (int16_t)roundf(yawFilt);
    motherCmd.pitch         = (int16_t)roundf(pitchFilt);
    motherCmd.roll          = (int16_t)roundf(rollFilt);
    motherCmd.hoverMode     = hoverPressed;
    motherCmd.emergencyStop = eStopHeld;

    // ---- Build mini drone packet ----
    // Mini drone is autonomous — sticks not needed, just arm/takeoff/land
    miniCmd.throttle      = THROTTLE_MIN_US;
    miniCmd.yaw           = 0;
    miniCmd.pitch         = 0;
    miniCmd.roll          = 0;
    miniCmd.arm           = miniArmed;
    miniCmd.takeoff       = isrTakeoffEdge;  // true for one loop only
    miniCmd.land          = isrLandEdge;     // true for one loop only
    miniCmd.emergencyStop = eStopHeld;

    // ---- Send both packets ----
    esp_now_send(motherdroneMAC, (uint8_t*)&motherCmd, sizeof(motherCmd));
    esp_now_send(miniDroneMAC,   (uint8_t*)&miniCmd,   sizeof(miniCmd));

    // ---- Clear edge flags after sending ----
    if (isrTakeoffEdge) {
        Serial.println(">>> Takeoff sent to mini drone");
        isrTakeoffEdge = false;
    }
    if (isrLandEdge) {
        Serial.println(">>> Land sent to mini drone");
        isrLandEdge = false;
    }

    // ---- Serial prompt ----
    if (droneFound && millis() - lastSerialPrompt > 3000) {
        Serial.println("Mini drone visible. Press LAND to land.");
        lastSerialPrompt = millis();
    }

    // ---- Serial status ----
    Serial.printf(
        "M:ARM%d LOCK%d HOV%d | T:%d Y:%d P:%d R:%d | "
        "m:ARM%d | FOUND:%d S:%s\n",
        motherArmed, throttleLocked, hoverPressed,
        throttleOut, (int)roundf(yawFilt),
        (int)roundf(pitchFilt), (int)roundf(rollFilt),
        miniArmed, droneFound, lastSendOK?"OK":"FAIL"
    );

    updateLCD(motherArmed, miniArmed);
    delay(LOOP_DT_MS);
}