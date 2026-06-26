#include "config.h"
#include "utils.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"
#include "receiver.h"
#include "wifi_comm.h"

// Define global variables that were declared as extern in config.h
const char* WIFI_SSID = "Esp32drone";
const char* WIFI_PASSWORD = "esp32drone";

void setup() {
  Serial.begin(SERIAL_BAUD);
  setFixedPIDGains();
  initializeWiFi();
  getUDP().begin(getUDPPort());
  Serial.print("UDP listening on port ");
  Serial.println(getUDPPort());
  read_MPU6050();
  setupMotors();

  // Optional: run MPU tests during initialization
  Serial.println("Run MPU tests? (g=go / any other key to skip)");
  if (Serial.available() && Serial.read() == 'g') {
      runMPUTests();
  }
  lastCommandTime = millis();
}

void loop() {
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    throttleValue = ThrottleIdle;
  }

  while (getUDP().parsePacket()) {
    int len = getUDP().read(getIncomingPacket(), 63);
    if (len > 0) {
      getIncomingPacket()[len] = '\0';
      Serial.print("UDP RX: "); Serial.println(getIncomingPacket());
      parseUDP(getIncomingPacket());
      // Print parsed values for quick debug
      Serial.print("Parsed -> throttle:"); Serial.print(throttleValue);
      Serial.print("  roll_cmd:"); Serial.print(target_angleRoll);
      Serial.print("  pitch_cmd:"); Serial.print(target_anglePitch);
      Serial.print("  yaw_rate_cmd:"); Serial.println(target_rateYaw);
    }
  }

  static unsigned long lastLoop = micros();
  static unsigned long lastStatus = millis();
  if (micros() - lastLoop >= LOOP_INTERVAL) {
    lastLoop += LOOP_INTERVAL;
    float dt = 0;
    readSensors(dt);
    if (dt > 0 && dt <= MAX_DT) {
      computePID(dt);
    }
    writeMotors();
  }

  // Periodic status print (every 500ms)
  if (millis() - lastStatus >= 500) {
    lastStatus = millis();
    Serial.print("STATUS: armed="); Serial.print(isArmed);
    Serial.print(" throttle="); Serial.print(throttleValue);
    Serial.print(" roll="); Serial.print(target_angleRoll);
    Serial.print(" pitch="); Serial.print(target_anglePitch);
    Serial.print(" yaw_rate="); Serial.println(target_rateYaw);
  }
}
