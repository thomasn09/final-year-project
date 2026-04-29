#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);   // IMPORTANT: ESP-NOW uses STA MAC
  delay(100);

  Serial.println("Getting MAC address...");
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}