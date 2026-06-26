#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

extern float target_angleRoll;
extern float target_anglePitch;
extern float target_rateYaw;

extern uint16_t motor[4];
extern bool isArmed;
extern int throttleValue;

void setupMotors();
void writeMotors();
void ArmTrue();
void ArmFalse();
// Motor test integrated into initialization; previous runMotorTest removed

#endif  // MOTORS_H
