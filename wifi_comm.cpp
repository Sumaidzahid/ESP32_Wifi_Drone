#include <Arduino.h>
#include "wifi_comm.h"

WiFiUDP udp;
const unsigned int udpPort = 4210;
char incomingPacket[64];

void initializeWiFi() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_AP);
  delay(WIFI_INIT_DELAY);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Access point started");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
}
