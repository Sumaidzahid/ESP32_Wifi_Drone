#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

extern uint16_t motor[4];
extern bool isArmed;
extern int throttleValue;
extern float target_angleRoll;
extern float target_anglePitch;
extern float target_rateYaw;

void setupMotors();
void writeMotors();
void ArmTrue();
void ArmFalse();

#endif  // MOTORS_H
