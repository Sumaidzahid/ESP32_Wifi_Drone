#ifndef PID_H
#define PID_H

#include "config.h"

struct PIDController {
  float kp, ki, kd;
  float prevError = 0;
  float integral = 0;

  float compute(float error, float dt);
  void reset();
};

extern PIDController anglePID_Roll, anglePID_Pitch;
extern PIDController ratePID_Roll, ratePID_Pitch, ratePID_Yaw;

extern float target_angleRoll, target_anglePitch, target_rateYaw;
extern float PIDrollOutput, PIDpitchOutput, PIDyawOutput;
extern float error_rateRoll, error_ratePitch, error_rateYaw;
extern float rollOutput, pitchOutput, yawOutput;
extern float pwmRollValue, pwmPitchValue, pwmYawValue;
extern float mix1, mix2, mix3, mix4;
extern float target_rateRoll, target_ratePitch;
extern float errorRoll, errorPitch;

void setFixedPIDGains();
void computePID(float dt);

#endif
