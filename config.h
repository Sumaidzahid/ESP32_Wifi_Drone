#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
const int MOTOR_MIN = 1000;           // Minimum motor PWM value
const int MOTOR_MAX = 2000;        // Maximum motor PWM value
const int MOTOR_PINS[4] = {7, 4, 2, 8};  // Motor pins {RL, FR, FL, RR}
const int PWM_FREQUENCY = 18000;   // PWM frequency in Hz
const int PWM_RESOLUTION = 10;     // 10-bit resolution (0-1023)

// ============================================================================
// ATTITUDE CONTROL
// ============================================================================
const float MAX_ATTITUDE_ANGLE = 15.0;  // Maximum pitch/roll angle command from user (degrees)
const float ROLL_GAIN = 1.2;       // Roll motor response gain
const float PITCH_GAIN = 1.2;      // Pitch motor response gain
const float YAW_GAIN = 1.2;        // Yaw motor response gain

// ============================================================================
// THROTTLE CONFIGURATION
// ============================================================================
const int ThrottleIdle = 1000;      // Idle throttle value (safe hover point)
const int THROTTLE_INCREMENT = 50; // Throttle change per command (smooth increments)

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define SDA_PIN 5                  // I2C SDA pin (MPU6050)
#define SCL_PIN 6                  // I2C SCL pin (MPU6050)
const int calibrationSamples = 1000;   // Number of samples for MPU calibration
const float COMPLEMENTARY_FILTER_ALPHA = 0.98;  // Complementary filter coefficient

// MPU6050 Sensor Ranges
#define MPU_ACCEL_RANGE MPU6050_RANGE_4_G   // ±4G accelerometer range
#define MPU_GYRO_RANGE MPU6050_RANGE_500_DEG // ±500°/s gyroscope range
#define MPU_FILTER_BW MPU6050_BAND_10_HZ     // 10Hz low-pass filter

// ============================================================================
// PID TUNING - ANGLE CONTROL (Attitude)
// ============================================================================
const float kp_Roll = 3.5;         // Roll angle proportional gain
const float ki_Roll = 0.02;        // Roll angle integral gain
const float kd_Roll = 1.2;         // Roll angle derivative gain

const float kp_Pitch = 3.5;        // Pitch angle proportional gain
const float ki_Pitch = 0.02;       // Pitch angle integral gain
const float kd_Pitch = 1.2;        // Pitch angle derivative gain

// ============================================================================
// PID TUNING - RATE CONTROL (Angular Velocity)
// ============================================================================
const float PRateRoll = 1.5;       // Roll rate proportional gain
const float IRateRoll = 0.01;      // Roll rate integral gain
const float DRateRoll = 0.08;      // Roll rate derivative gain

const float PRatePitch = 1.5;      // Pitch rate proportional gain
const float IRatePitch = 0.01;     // Pitch rate integral gain
const float DRatePitch = 0.08;     // Pitch rate derivative gain

const float PRateYaw = 0.5;        // Yaw rate proportional gain
const float IRateYaw = 0.01;       // Yaw rate integral gain
const float DRateYaw = 0.05;       // Yaw rate derivative gain

// ============================================================================
// PID INTEGRAL LIMITS (Anti-Windup)
// ============================================================================
const float PID_INTEGRAL_MAX = 200.0;   // Maximum integral accumulation

// ============================================================================
// FAILSAFE CONFIGURATION
// ============================================================================
const unsigned long COMMAND_TIMEOUT = 2000;  // Disarm if no command for 2 seconds (ms)
const float MAX_DT = 0.1;          // Maximum allowed dt (100ms) - skip if exceeded
const float MIN_DT = 0.0001;       // Minimum allowed dt (0.1ms) - skip if below

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
const char* WIFI_SSID = "Esp32drone";
const char* WIFI_PASSWORD = "esp32drone";
const int WIFI_PORT = 80;

// ============================================================================
// SERIAL CONFIGURATION
// ============================================================================
const long SERIAL_BAUD = 115200;   // Serial communication baud rate

// ============================================================================
// ADDITIONAL I2C & MPU6050 CONFIGURATION
// ============================================================================
const int I2C_CLOCK_SPEED = 100000;  // I2C clock speed in Hz (400kHz = Fast-mode)
const int I2C_BUS_NUMBER = 0;      // I2C bus number (0 = default I2C bus)
const uint8_t MPU_I2C_ADDRESS = 0x68;  // MPU6050 I2C slave address

// ============================================================================
// MPU6050 RAW DATA SCALING CONSTANTS
// ============================================================================
const float MPU6050_ACCEL_SCALE = 16384.0;  // Accelerometer raw to g conversion
const float MPU6050_GYRO_SCALE = 131.0;     // Gyroscope raw to °/s conversion

// ============================================================================
// CALIBRATION & SENSOR DELAYS
// ============================================================================
const int CALIBRATION_DELAY_MS = 5;    // Delay between calibration samples (ms) for stable readings
const float GRAVITY_CONSTANT = 9.81;   // Earth's gravitational acceleration (m/s²)
const int WIFI_INIT_DELAY = 100;    // Delay after WiFi mode change (ms)
const int MPU_INIT_DELAY = 1000;    // Delay for MPU6050 power-up stabilization (ms)
const int SERIAL_WAIT_DELAY = 10;   // Delay while waiting for serial connection (ms)
const int ERROR_RESTART_DELAY = 10; // Delay in error loop before restart check (ms)

// ============================================================================
// YAW ANGLE WRAPPING
// ============================================================================
const float YAW_ANGLE_MAX = 180.0;   // Maximum yaw angle before wrapping (degrees)
const float YAW_ANGLE_MIN = -180.0;  // Minimum yaw angle before wrapping (degrees)

// ============================================================================
// PWM OUTPUT LIMITS (Constraint Ranges)
// ============================================================================
const int PWM_OUTPUT_MIN = -1000;   // Minimum PWM adjustment value (full negative correction)
const int PWM_OUTPUT_MAX = 1000;    // Maximum PWM adjustment value (full positive correction)
const int PWM_MOTOR_OUTPUT_MIN = 0;     // Minimum motor PWM (0 = off)
const int PWM_MOTOR_OUTPUT_MAX = 1023;  // Maximum motor PWM (1023 = full 10-bit range)

// ============================================================================
// ANGLE CONSTRAINTS
// ============================================================================

#endif  // CONFIG_H
