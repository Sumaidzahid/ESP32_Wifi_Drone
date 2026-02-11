#include "motors.h"

uint16_t motor[4];
bool isArmed = false;
int throttleValue = 0;
float target_angleRoll = 0, target_anglePitch = 0, target_rateYaw = 0;

void setupMotors() {
  pinMode(MOTOR_PINS[0], OUTPUT);
  pinMode(MOTOR_PINS[1], OUTPUT);
  pinMode(MOTOR_PINS[2], OUTPUT);
  pinMode(MOTOR_PINS[3], OUTPUT);

  ledcAttach(MOTOR_PINS[0], PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_PINS[1], PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_PINS[2], PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR_PINS[3], PWM_FREQUENCY, PWM_RESOLUTION);

  // Initialize motor outputs to 0
  ledcWrite(MOTOR_PINS[0], 0);
  ledcWrite(MOTOR_PINS[1], 0);
  ledcWrite(MOTOR_PINS[2], 0);
  ledcWrite(MOTOR_PINS[3], 0);

  Serial.println("✅ Motor PWM initialized successfully");
}

void writeMotors() {
  ledcWrite(MOTOR_PINS[0], motor[0]);
  ledcWrite(MOTOR_PINS[1], motor[1]);
  ledcWrite(MOTOR_PINS[2], motor[2]);
  ledcWrite(MOTOR_PINS[3], motor[3]);
}

void ArmTrue() {
  isArmed = true;
}

void ArmFalse() {
  isArmed = false;
  throttleValue = ThrottleIdle;
  target_angleRoll = 0;
  target_anglePitch = 0;
  target_rateYaw = 0;
}
