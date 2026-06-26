#include "motors.h"
#include "utils.h"

uint16_t motor[4] = {PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MIN};
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

  // Initialize motor outputs to safe minimum
  ledcWrite(MOTOR_PINS[0], PWM_MOTOR_OUTPUT_MIN);
  ledcWrite(MOTOR_PINS[1], PWM_MOTOR_OUTPUT_MIN);
  ledcWrite(MOTOR_PINS[2], PWM_MOTOR_OUTPUT_MIN);
  ledcWrite(MOTOR_PINS[3], PWM_MOTOR_OUTPUT_MIN);

  // Integrated motor test: run each motor at test value (300) for 2s during initialization
  Serial.println("Running integrated motor test: each motor will run at test value for 2s...");
  int lowCmd = MOTOR_TEST_VALUE; 
  for (int m = 0; m < 4; ++m) {
    Serial.print("Init test - motor "); Serial.println(m);
    // ensure all motors at minimum first
    for (int i=0;i<4;i++) {
      motor[i] = PWM_MOTOR_OUTPUT_MIN;
      ledcWrite(MOTOR_PINS[i], PWM_MOTOR_OUTPUT_MIN);
    }
    uint16_t testPWM = (uint16_t) constrain(mapFloat(lowCmd, 0, MOTOR_MAX, PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX), PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX);
    Serial.print(" -> test PWM: "); Serial.println(testPWM);
    ledcWrite(MOTOR_PINS[m], testPWM);
    delay(MOTOR_TEST_DURATION_MS);
    // stop
    ledcWrite(MOTOR_PINS[m], PWM_MOTOR_OUTPUT_MIN);
    motor[m] = PWM_MOTOR_OUTPUT_MIN;
    delay(MOTOR_TEST_PAUSE_MS);
  }
  Serial.println("Integrated motor test complete.");
  
  // Ensure internal state is in a safe, disarmed configuration
  isArmed = false;
  throttleValue = ThrottleIdle;
  for (int i = 0; i < 4; ++i) motor[i] = PWM_MOTOR_OUTPUT_MIN;

  Serial.println("✅ Motor PWM initialized successfully");
}

void writeMotors() {
  // If disarmed, always keep motors at safe minimum output
  if (!isArmed) {
    ledcWrite(MOTOR_PINS[0], PWM_MOTOR_OUTPUT_MIN);
    ledcWrite(MOTOR_PINS[1], PWM_MOTOR_OUTPUT_MIN);
    ledcWrite(MOTOR_PINS[2], PWM_MOTOR_OUTPUT_MIN);
    ledcWrite(MOTOR_PINS[3], PWM_MOTOR_OUTPUT_MIN);
    return;
  }

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
  // Immediately force motor outputs to safe minimum
  for (int i = 0; i < 4; ++i) motor[i] = PWM_MOTOR_OUTPUT_MIN;
  writeMotors();
}

// motor test removed; behavior integrated into setupMotors()
