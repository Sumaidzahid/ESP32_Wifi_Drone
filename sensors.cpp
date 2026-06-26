#include <Arduino.h>
#include "sensors.h"

MPU6050 mpu(MPU_I2C_ADDRESS);
SensorState sensors;    
static float accErrorX = 0, accErrorY = 0, accErrorZ = 0;
static float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

TwoWire I2CBus = TwoWire(0);

void read_MPU6050() {
  while (!Serial) delay(SERIAL_WAIT_DELAY);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  // Ensure the default Wire bus is used (matches the working minimal sketch)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED);
//   // Also initialize project TwoWire instance in case other modules use it
//   I2CBus.begin(SDA_PIN, SCL_PIN);
//   I2CBus.setClock(I2C_CLOCK_SPEED);

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
    // Seed sensor filters by reading a few samples so roll/pitch/yaw aren't stuck at 0
    {
      float dt = 0;
      for (int i = 0; i < 10; ++i) {
        readSensors(dt);
        delay(20);
      }
    }
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

float readSensors(float &dt) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  sensors.accX = (ax / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorX;
  sensors.accY = (ay / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorY;
  sensors.accZ = (az / MPU6050_ACCEL_SCALE * GRAVITY_CONSTANT) - accErrorZ;

  sensors.gyroX = (gx / MPU6050_GYRO_SCALE) - gyroErrorX;
  sensors.gyroY = (gy / MPU6050_GYRO_SCALE) - gyroErrorY;
  sensors.gyroZ = (gz / MPU6050_GYRO_SCALE) - gyroErrorZ;

  float accelRoll = atan2(sensors.accY, copysign(sqrt(sensors.accZ*sensors.accZ + sensors.accX*sensors.accX), sensors.accZ)) * 180.0 / PI;
  float accelPitch = atan2(-sensors.accX, copysign(sqrt(sensors.accY*sensors.accY + sensors.accZ*sensors.accZ), sensors.accZ)) * 180.0 / PI;

  static unsigned long lastTime = micros();
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) * 1e-6;
  lastTime = currentTime;

  if (dt <= 0 || dt > MAX_DT) return dt;  // Skip if dt is invalid

  sensors.roll = COMPLEMENTARY_FILTER_ALPHA * (sensors.roll + sensors.gyroX * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accelRoll;
  sensors.pitch = COMPLEMENTARY_FILTER_ALPHA * (sensors.pitch + sensors.gyroY * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;
  static float yaw = 0;
  yaw += sensors.gyroZ * dt;
  if (yaw > YAW_ANGLE_MAX) yaw -= 360;
  else if (yaw < YAW_ANGLE_MIN) yaw += 360;

  // Publish yaw into shared sensor state
  sensors.yaw = yaw;

  return dt;
}

// Interactive MPU tests: rotation tests per-axis and a freestyle 10s test
void runMPUTests() {
  Serial.println("\n=== MPU TESTS ===");
  Serial.println("Rotation test: for each axis rotate to ~30deg; live values will be shown and detection will prompt you to continue.");
  float dt = 0;
  const char axes[3] = {'R','P','Y'}; // Roll, Pitch, Yaw

  for (int a = 0; a < 3; ++a) {
    const char axis = axes[a];
    Serial.print("Rotate ");
    if (axis == 'R') Serial.print("ROLL");
    else if (axis == 'P') Serial.print("PITCH");
    else Serial.print("YAW");
    Serial.print(" to approx "); Serial.print(MPU_TEST_ANGLE); Serial.println(" degrees.");
    Serial.println("Live readings will display; auto-advances when target angle reached (or times out).");

    bool reached = false;
    unsigned long lastPrint = 0;
    unsigned long startTime = millis();
    const unsigned long timeoutMs = MPU_TEST_TIMEOUT_MS;

    while (millis() - startTime < timeoutMs) {
      readSensors(dt);
      float current = 0;
      if (axis == 'R') current = sensors.roll;
      else if (axis == 'P') current = sensors.pitch;
      else current = sensors.yaw;

      unsigned long now = millis();
      if (now - lastPrint >= MPU_LIVE_PRINT_MS) {
        Serial.print("Live ");
        if (axis == 'R') Serial.print("Roll: ");
        else if (axis == 'P') Serial.print("Pitch: ");
        else Serial.print("Yaw: ");
        Serial.println(current);
        lastPrint = now;
      }

      if (fabs(current - MPU_TEST_ANGLE) <= MPU_TEST_TOLERANCE) {
        // capture a short average for reporting
        float sum = 0; int cnt = 0;
        for (int i = 0; i < 10; ++i) { readSensors(dt); if (axis == 'R') sum += sensors.roll; else if (axis == 'P') sum += sensors.pitch; else sum += sensors.yaw; cnt++; delay(20); }
        float avg = sum / cnt;
        Serial.print("Captured average for ");
        if (axis == 'R') Serial.print("Roll: "); else if (axis == 'P') Serial.print("Pitch: "); else Serial.print("Yaw: ");
        Serial.println(avg);
        reached = true;
        break;
      }

    }

    if (!reached) {
      Serial.println("Timeout waiting for target angle; proceeding to next axis.");
    } else {
      Serial.println("Proceeding to next axis...");
    }
  }

  Serial.println("=== MPU TESTS COMPLETE ===\n");
}
