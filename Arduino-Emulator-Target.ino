/**
 * Arduino-Simulator Execution Target - This code receives commands from the remote Arduino-Simulator.
 * Executes them and returns the result back to the requestor using UDP.
 * 
 * (c) 2020 Phil Schatzmann
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "HardwareService.h"
#include "CommandHandler.h"


const char* wifi_ssid = "Phil Schatzmann";
const char* wifi_pwd = "sabrina01";
const char* udpAddress = "192.168.255.255"; // we use a broadcast address
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

void setupConnection() {
    // send out identification "Arduino-Emulator" and expect an "OK" string
    while(true) {
      Logger.log(Info,"Waiting for hallo from remote Arduino-Emulator");
      udp.beginPacket(udpAddress,udpPort);
      udp.print("Arduino-Emulator");
      udp.endPacket();
      delay(5000);

      char reply[3];
      int len = udp.read(reply,2);
      reply[len]=0;
      if (strcmp(reply,"OK")==0){
        Logger.log(Info,"Hallo from Emulator received - ready for commands...");
        break;
      } 
    }
}

void setup() {
    Serial.begin(115200);
    Logger.setLogger(Serial, Info);
    delay(10);

    setupWiFi();
    udp.begin(udpPort);
    //setupConnection();
}


void loop() {
    udp.parsePacket();
    if (udp.available()>0) {
      handler.receiveCommand(service);
      // we should not have any unprocessed data;
      // but we get rid of it....
      char msg[512];
      udp.read(msg,512);
    }
}
