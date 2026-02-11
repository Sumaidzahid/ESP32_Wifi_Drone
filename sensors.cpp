#include <Arduino.h>
#include "sensors.h"

MPU6050 mpu;
float accErrorX = 0, accErrorY = 0, accErrorZ = 0;
float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;
float roll = 0, pitch = 0;
TwoWire I2CBus = TwoWire(0);

void read_MPU6050() {
  while (!Serial) delay(SERIAL_WAIT_DELAY);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  I2CBus.begin(SDA_PIN, SCL_PIN);
  I2CBus.setClock(I2C_CLOCK_SPEED);

  delay(MPU_INIT_DELAY);

  Serial.println("Initializing MPU6050...");

  mpu.initialize();
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  mpu.setSleepEnabled(false);

  if (mpu.testConnection()) {
    Serial.println("✅ MPU6050 initialized! Starting calibration...");
    calibrateMPU();
    Serial.println("Calibration complete.");
  } else {
    Serial.println("❌ MPU6050 initialization failed on ESP32-S3!");
    while (1) delay(ERROR_RESTART_DELAY);
  }
}

void calibrateMPU() {
  float accX = 0, accY = 0, accZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accX += (ax / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT);
    accY += (ay / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT);
    accZ += (az / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - GRAVITY_CONSTANT;

    gyroX += (gx / MPU6050_GYRO_SCALE);
    gyroY += (gy / MPU6050_GYRO_SCALE);
    gyroZ += (gz / MPU6050_GYRO_SCALE);

    delay(CALIBRATION_DELAY_MS);
  }

  accErrorX = accX / calibrationSamples;
  accErrorY = accY / calibrationSamples;
  accErrorZ = accZ / calibrationSamples;

  gyroErrorX = gyroX / calibrationSamples;
  gyroErrorY = gyroY / calibrationSamples;
  gyroErrorZ = gyroZ / calibrationSamples;

  Serial.println("Calibration Offsets:");
  Serial.print("Accelerometer Errors (m/s²): X=");
  Serial.print(accErrorX);
  Serial.print(", Y=");
  Serial.print(accErrorY);
  Serial.print(", Z=");
  Serial.println(accErrorZ);

  Serial.print("Gyroscope Errors (°/s): X=");
  Serial.print(gyroErrorX);
  Serial.print(", Y=");
  Serial.print(gyroErrorY);
  Serial.print(", Z=");
  Serial.println(gyroErrorZ);
}

float readSensors() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelerationX = (ax / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorX;
  float accelerationY = (ay / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorY;
  float accelerationZ = (az / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorZ;

  gyroX = (gx / MPU6050_GYRO_SCALE) - gyroErrorX;
  gyroY = (gy / MPU6050_GYRO_SCALE) - gyroErrorY;
  gyroZ = (gz / MPU6050_GYRO_SCALE) - gyroErrorZ;

  float AccelRoll = atan2(accelerationY, copysign(sqrt(accelerationZ*accelerationZ + accelerationX*accelerationX), accelerationZ)) * 180/PI;
  float denominator = copysign(sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ), accelerationZ);
  float AccelPitch = atan2(-accelerationX, denominator) * 180 / PI;

  static unsigned long lastTime = micros();
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) * 1e-6;
  lastTime = currentTime;

  if (dt <= 0 || dt > MAX_DT) return dt;  // Skip if dt is invalid

  roll = COMPLEMENTARY_FILTER_ALPHA * (roll + gyroX*dt) + (1-COMPLEMENTARY_FILTER_ALPHA) * AccelRoll;
  pitch = COMPLEMENTARY_FILTER_ALPHA * (pitch + gyroY*dt) + (1-COMPLEMENTARY_FILTER_ALPHA) * AccelPitch;

  static float yaw = 0;
  yaw += gyroZ * dt;
  if (yaw > YAW_ANGLE_MAX) yaw -= 360;
  else if (yaw < YAW_ANGLE_MIN) yaw += 360;

  return dt;
}
