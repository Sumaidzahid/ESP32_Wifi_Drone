#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

inline WiFiUDP& getUDP() {
    static WiFiUDP udp;
    return udp;
}

inline const unsigned int& getUDPPort() {
    static const unsigned int udpPort = 4210;
    return udpPort;
}

inline char* getIncomingPacket() {
    static char incomingPacket[64];
    return incomingPacket;
}

inline void initializeWiFi() {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_AP);
    delay(WIFI_INIT_DELAY);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Access point started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

#endif
