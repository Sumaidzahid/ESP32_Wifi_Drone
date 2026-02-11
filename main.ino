#include "config.h"
#include "utils.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"
#include "receiver.h"
#include "wifi_comm.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  setFixedPIDGains();
  initializeWiFi();
  udp.begin(udpPort);
  Serial.print("UDP listening on port ");
  Serial.println(udpPort);
  read_MPU6050();
  setupMotors();
  lastCommandTime = millis();
}

void loop() {
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    throttleValue = ThrottleIdle;
  }

  while (udp.parsePacket()) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
      parseUDP(incomingPacket);
    }
  }

  static unsigned long lastLoop = micros();
  if (micros() - lastLoop >= LOOP_INTERVAL) {
    lastLoop += LOOP_INTERVAL;
    float dt = readSensors();
    if (dt > 0 && dt <= MAX_DT) {
      computePID(dt);
    }
    writeMotors();
  }
}
