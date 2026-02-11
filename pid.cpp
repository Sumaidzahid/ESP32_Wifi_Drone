#include <Arduino.h>
#include "pid.h"
#include "sensors.h"
#include "motors.h"
#include "utils.h"

PIDController anglePID_Roll, anglePID_Pitch;
PIDController ratePID_Roll, ratePID_Pitch, ratePID_Yaw;

float target_angleRoll = 0, target_anglePitch = 0, target_rateYaw = 0;
float PIDrollOutput = 0, PIDpitchOutput = 0, PIDyawOutput = 0;
float error_rateRoll = 0, error_ratePitch = 0, error_rateYaw = 0;
float rollOutput = 0, pitchOutput = 0, yawOutput = 0;
float pwmRollValue = 0, pwmPitchValue = 0, pwmYawValue = 0;
float mix1 = 0, mix2 = 0, mix3 = 0, mix4 = 0;
float target_rateRoll = 0, target_ratePitch = 0;
float errorRoll = 0, errorPitch = 0;

float PIDController::compute(float error, float dt) {
  float p = kp * error;
  integral += ki * (error + prevError) * (dt / 2);
  integral = constrain(integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
  float i = integral;
  float d = (dt > 0) ? kd * (error - prevError) / dt : 0;
  prevError = error;
  return p + i + d;
}

void PIDController::reset() {
  prevError = 0;
  integral = 0;
}

void setFixedPIDGains() {
  anglePID_Roll.kp = kp_Roll;
  anglePID_Roll.ki = ki_Roll;
  anglePID_Roll.kd = kd_Roll;

  anglePID_Pitch.kp = kp_Pitch;
  anglePID_Pitch.ki = ki_Pitch;
  anglePID_Pitch.kd = kd_Pitch;

  ratePID_Roll.kp = PRateRoll;
  ratePID_Roll.ki = IRateRoll;
  ratePID_Roll.kd = DRateRoll;

  ratePID_Pitch.kp = PRatePitch;
  ratePID_Pitch.ki = IRatePitch;
  ratePID_Pitch.kd = DRatePitch;

  ratePID_Yaw.kp = PRateYaw;
  ratePID_Yaw.ki = IRateYaw;
  ratePID_Yaw.kd = DRateYaw;
}

void computePID(float dt) {
  // Angle PID
  errorRoll = target_angleRoll - roll;
  target_rateRoll = anglePID_Roll.compute(errorRoll, dt);

  errorPitch = target_anglePitch - pitch;
  target_ratePitch = anglePID_Pitch.compute(errorPitch, dt);

  // Rate PID
  error_rateRoll = target_rateRoll - gyroX;  // Need gyroX from sensors
  rollOutput = ratePID_Roll.compute(error_rateRoll, dt);

  error_ratePitch = target_ratePitch - gyroY;
  pitchOutput = ratePID_Pitch.compute(error_ratePitch, dt);

  error_rateYaw = target_rateYaw - gyroZ;
  yawOutput = ratePID_Yaw.compute(error_rateYaw, dt);

  // Scale outputs
  pwmRollValue = constrain(rollOutput * ROLL_GAIN, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX);
  pwmPitchValue = constrain(pitchOutput * PITCH_GAIN, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX);
  pwmYawValue = constrain(yawOutput * YAW_GAIN, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX);

  // Motor mixing
  mix1 = constrain(throttleValue + pwmRollValue + pwmPitchValue - pwmYawValue, MOTOR_MIN, MOTOR_MAX);
  mix2 = constrain(throttleValue - pwmRollValue - pwmPitchValue - pwmYawValue, MOTOR_MIN, MOTOR_MAX);
  mix3 = constrain(throttleValue + pwmRollValue - pwmPitchValue + pwmYawValue, MOTOR_MIN, MOTOR_MAX);
  mix4 = constrain(throttleValue - pwmRollValue + pwmPitchValue + pwmYawValue, MOTOR_MIN, MOTOR_MAX);

  // PWM output mapping
  motor[0] = (uint16_t) mapFloat(mix1, MOTOR_MIN, MOTOR_MAX, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX);
  motor[1] = (uint16_t) mapFloat(mix2, MOTOR_MIN, MOTOR_MAX, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX);
  motor[2] = (uint16_t) mapFloat(mix3, MOTOR_MIN, MOTOR_MAX, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX);
  motor[3] = (uint16_t) mapFloat(mix4, MOTOR_MIN, MOTOR_MAX, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX);
}
