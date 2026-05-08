/**
 * Arduino-Emulator execution target example.
 * Receives commands from a remote Arduino-Emulator and executes them.
 */

#include <EmulatorTarget.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* wifi_ssid = "YOUR_WIFI_SSID";
const char* wifi_pwd = "YOUR_WIFI_PASSWORD";
const int udpPort = 7000;

WiFiUDP udp;
HardwareService service(udp);
CommandHandler handler;

void setupWiFi() {
  Serial.println();
  Serial.println();
  Serial.print("Connecting to Network...");

  WiFi.begin(wifi_ssid, wifi_pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  EmulatorLogger.setLogger(Serial, Info);
  delay(10);

  setupWiFi();
  udp.begin(udpPort);
}

void loop() {
  udp.parsePacket();
  if (udp.available() > 0) {
    handler.receiveCommand(service);
    // Remove potential remaining payload data.
    char msg[512];
    udp.read(msg, sizeof(msg));
  }
}
