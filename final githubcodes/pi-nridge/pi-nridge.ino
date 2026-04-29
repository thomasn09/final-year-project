/*
 * pi_bridge.ino — ESP32 Pi Bridge
 * ─────────────────────────────────────────────────────────────────
 * Receives AprilTag data from the Pi detection.py script over serial,
 * then forwards a DroneStatusPacket to the controller via ESP-NOW.
 *

 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ── Pins ──────────────────────────────────────────────────────────
#define PI_RX_PIN      16    // ← Pi USB-serial TX
#define PI_TX_PIN      17    // → Pi (unused, keeps Serial2 happy)
#define PI_BAUD        115200
#define ESPNOW_CHANNEL 1

// Pi camera frame dimensions (must match detection.py)
#define FRAME_W  1280
#define FRAME_H   720

// ── Controller MAC ────────────────────────────────────────────────
// Paste the controller's MAC here after reading it from its serial monitor
uint8_t controllerMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // ← update

// ── Shared packet — MUST match controller.ino exactly ─────────────
#pragma pack(push, 1)
struct DroneStatusPacket {
    bool     droneSeen;   // true = tag currently visible
    float    tagX;        // normalised -1..+1  (
    float    tagY;        // normalised -1..+1 
    uint16_t tagSize;     // pixel side-length — bigger means closer
    int16_t  tagAngle;    // tag rotation in degrees
    uint8_t  tagId;       // which AprilTag
};
#pragma pack(pop)

// ── State ─────────────────────────────────────────────────────────
DroneStatusPacket statusPkt;
String            rxLine     = "";
unsigned long     lastSeenMs = 0;
const uint32_t    SEEN_TIMEOUT_MS = 400;  // ms silence from Pi → tag lost

bool espNowReady = false;

// ── Parse one line from Pi ────────────────────────────────────────
// Expected:  "tag_id,cx,cy,size,angle"
void parseLine(const String& line) {
    // Must contain exactly 4 commas
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);

    if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0) return;  // malformed

    int     tagId = line.substring(0,      c1).toInt();
    int     cx    = line.substring(c1 + 1, c2).toInt();
    int     cy    = line.substring(c2 + 1, c3).toInt();
    int     size  = line.substring(c3 + 1, c4).toInt();
    int     angle = line.substring(c4 + 1).toInt();

    // Sanity check pixel range
    if (cx < 0 || cx > FRAME_W || cy < 0 || cy > FRAME_H) return;

    // Normalise cx/cy to -1..+1 from frame centre
    float nx = (float)(cx - FRAME_W / 2) / (float)(FRAME_W / 2);
    float ny = (float)(cy - FRAME_H / 2) / (float)(FRAME_H / 2);

    statusPkt.droneSeen = true;
    statusPkt.tagId     = (uint8_t)  tagId;
    statusPkt.tagX      = nx;
    statusPkt.tagY      = ny;
    statusPkt.tagSize   = (uint16_t) constrain(size,  0, 65535);
    statusPkt.tagAngle  = (int16_t)  constrain(angle, -180, 180);

    lastSeenMs = millis();
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== PI BRIDGE ===");

    // Pi data arrives via USB (ttyUSB0 on Pi side = Serial on ESP32 side)
    // Serial is already started above at 115200 — same baud, no extra begin needed
    Serial.println("Reading Pi data from USB serial (same port as debug)");

    // ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("Bridge MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println(">>> Paste this MAC into controller.ino as piBridgeMAC[] <<<\n");

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW FAILED — halting");
        while (true) delay(10);
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, controllerMAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("Controller peer registered");
    } else {
        Serial.println("WARNING: peer add failed — will use broadcast");
        // Fall back to broadcast so testing still works before MAC is set
        uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        memcpy(peer.peer_addr, broadcast, 6);
        esp_now_add_peer(&peer);
        memcpy(controllerMAC, broadcast, 6);
    }

    espNowReady = true;
    Serial.println("Pi Bridge ready — waiting for Pi serial data\n");
}

// ── Loop ──────────────────────────────────────────────────────────
unsigned long lastSendMs  = 0;
unsigned long lastDebugMs = 0;

void loop() {
    // ── Read serial lines from Pi ──
    while (Serial.available()) {
        char c = (char) Serial.read();
        if (c == '\n') {
            rxLine.trim();
            if (rxLine.length() > 0) {
                parseLine(rxLine);
                rxLine = "";
            }
        } else if (c != '\r') {
            rxLine += c;
            if (rxLine.length() > 80) rxLine = "";  // overflow guard
        }
    }

    // ── Timeout — Pi sends nothing when no tag is visible ──
    if (statusPkt.droneSeen && (millis() - lastSeenMs > SEEN_TIMEOUT_MS)) {
        statusPkt.droneSeen = false;
        statusPkt.tagX      = 0.0f;
        statusPkt.tagY      = 0.0f;
        Serial.println("Tag lost");
    }

    // ── Send to controller at 20 Hz ──
    if (espNowReady && (millis() - lastSendMs >= 50)) {
        lastSendMs = millis();
        esp_now_send(controllerMAC, (uint8_t*) &statusPkt, sizeof(statusPkt));
    }

    // Debug removed — Serial is shared with Pi, printing here would corrupt the data stream
    // To debug, watch the controller LCD or C3 serial monitor instead
}