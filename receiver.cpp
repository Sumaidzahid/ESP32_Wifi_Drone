#include <Arduino.h>
#include "receiver.h"
#include "motors.h"

unsigned long lastCommandTime = 0;

void parseUDP(char* data) {
  size_t msgLen = strlen(data);
  while (msgLen > 0 && (data[msgLen - 1] == '\r' || data[msgLen - 1] == '\n' || data[msgLen - 1] == ' ')) {
    data[--msgLen] = '\0';
  }

  if (strcmp(data, "ARMED:true") == 0) {
    ArmTrue();
    lastCommandTime = millis();
    return;
  }

  if (strcmp(data, "ARMED:false") == 0) {
    ArmFalse();
    lastCommandTime = millis();
    return;
  }

  float tNorm, rNorm, pNorm, yNorm;

  float tf, rf, pf, yf;
  int ti, ri, pi, yi;

  if (sscanf(data, "%f,%f,%f,%f", &tf, &rf, &pf, &yf) == 4 ||
      sscanf(data, "T:%f,R:%f,P:%f,Y:%f", &tf, &rf, &pf, &yf) == 4) {
    tNorm = constrain(tf, 0.0f, 1.0f);
    rNorm = constrain(rf, -1.0f, 1.0f);
    pNorm = constrain(pf, -1.0f, 1.0f);
    yNorm = constrain(yf, -1.0f, 1.0f);
  } else if (sscanf(data, "T:%d,R:%d,P:%d,Y:%d", &ti, &ri, &pi, &yi) == 4) {
    tNorm = constrain((float)ti / 100.0f, 0.0f, 1.0f);
    rNorm = constrain((float)ri / 100.0f, -1.0f, 1.0f);
    pNorm = constrain((float)pi / 100.0f, -1.0f, 1.0f);
    yNorm = constrain((float)yi / 100.0f, -1.0f, 1.0f);
  } else {
    Serial.println("Invalid UDP packet format");
    return;
  }

  throttleValue = (int)(ThrottleIdle + tNorm * (MOTOR_MAX - ThrottleIdle));
  target_angleRoll = rNorm * MAX_ATTITUDE_ANGLE;
  target_anglePitch = pNorm * MAX_ATTITUDE_ANGLE;
  target_rateYaw = yNorm * MAX_ATTITUDE_ANGLE;

  lastCommandTime = millis();
}

void goForward() {
  target_anglePitch = MAX_ATTITUDE_ANGLE;
}
void goforward() {
  target_anglePitch = 0;
}
void goBack() {
  target_anglePitch = -MAX_ATTITUDE_ANGLE;
}
void goback() {
  target_anglePitch = 0;
}
void goLeft() {
  target_angleRoll = -MAX_ATTITUDE_ANGLE;
}
void goleft() {
  target_angleRoll = 0;
}
void goRight() {
  target_angleRoll = MAX_ATTITUDE_ANGLE;
}
void goright() {
  target_angleRoll = 0;
}
void rotateRight() {
  target_rateYaw = MAX_ATTITUDE_ANGLE;
}
void rotateLeft() {
  target_rateYaw = -MAX_ATTITUDE_ANGLE;
}
void rotateright() {
  target_rateYaw = 0;
}
void rotateleft() {
  target_rateYaw = 0;
}
void goUp() {
  if(isArmed) {
    throttleValue = min(throttleValue + THROTTLE_INCREMENT, MOTOR_MAX);
  }
}
void goDown() {
  if(isArmed) {
    throttleValue = max(throttleValue - THROTTLE_INCREMENT, ThrottleIdle);
  }
}
