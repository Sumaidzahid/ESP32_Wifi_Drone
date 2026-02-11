#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

extern WiFiUDP udp;
extern const unsigned int udpPort;
extern char incomingPacket[64];

void initializeWiFi();

#endif
