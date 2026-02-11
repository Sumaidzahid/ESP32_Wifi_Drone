# Configuration Constants Audit - Post-Modularization Update

## Summary
Following the modularization of the ESP32 drone code into separate files (main.ino, motors.h/cpp, sensors.h/cpp, pid.h/cpp, receiver.h/cpp, wifi_comm.h/cpp, utils.h/cpp), this audit has been updated to reflect the new file locations where constants are used. All hardcoded constants remain in `config.h` with comprehensive comments for easy understanding and maintenance. The modular structure improves code organization without affecting performance.

---

## All Constants Found & Migrated

### 1. **MOTOR & PWM Configuration**
| Constant | Value | Purpose | Location |
|----------|-------|---------|----------|
| `MOTOR_MIN` | 1000 | Minimum PWM output | Motor mixing in pid.cpp |
| `MOTOR_MAX` | 2000 | Maximum PWM output | Motor mixing in pid.cpp |
| `MOTOR_PINS[4]` | {7,4,2,8} | GPIO pins for motors | Motor initialization in motors.cpp |
| `PWM_FREQUENCY` | 18000 Hz | PWM frequency | Motor setup in motors.cpp |
| `PWM_RESOLUTION` | 10 bits | PWM resolution (0-1023) | Motor setup in motors.cpp |
| `PWM_OUTPUT_MIN` | -1000 | Min PID correction | PID scaling in pid.cpp |
| `PWM_OUTPUT_MAX` | 1000 | Max PID correction | PID scaling in pid.cpp |
| `PWM_MOTOR_OUTPUT_MIN` | 0 | Min motor PWM value | Motor output mapping in pid.cpp |
| `PWM_MOTOR_OUTPUT_MAX` | 1023 | Max motor PWM value | Motor output mapping in pid.cpp |

### 2. **Attitude Control (Gains)**
| Constant | Value | Purpose |
|----------|-------|---------|
| `ROLL_GAIN` | 1.2 | Roll correction scaling |
| `PITCH_GAIN` | 1.2 | Pitch correction scaling |
| `YAW_GAIN` | 1.2 | Yaw correction scaling |
| `MAX_ATTITUDE_ANGLE` | 15.0° | Max pitch/roll command |

### 3. **Throttle Control**
| Constant | Value | Purpose |
|----------|-------|---------|
| `ThrottleIdle` | 1000 | Safe hover throttle (~15%) |
| `THROTTLE_INCREMENT` | 50 | Smooth U/D command steps |

### 4. **I2C & MPU6050 Sensor Configuration**
| Constant | Value | Purpose | Source |
|----------|-------|---------|--------|
| `SDA_PIN` | 5 | I2C data pin | GPIO 5 in sensors.cpp |
| `SCL_PIN` | 6 | I2C clock pin | GPIO 6 in sensors.cpp |
| `I2C_CLOCK_SPEED` | 100000 Hz | I2C standard-mode | `I2CBus.setClock()` in sensors.cpp |
| `I2C_BUS_NUMBER` | 0 | I2C bus index | `TwoWire(0)` in sensors.cpp |
| `MPU_I2C_ADDRESS` | 0x68 | MPU6050 slave address | `mpu.initialize()` in sensors.cpp |

### 5. **Sensor Calibration**
| Constant | Value | Purpose |
|----------|-------|---------|
| `calibrationSamples` | 1000 | Samples for offset calc |
| `CALIBRATION_DELAY_MS` | 5 ms | Delay between samples |
| `GRAVITY_CONSTANT` | 9.81 m/s² | Earth gravity reference |

### 6. **Sensor Ranges (MPU6050)**
| Define | Value | Purpose |
|--------|-------|---------|
| `MPU_ACCEL_RANGE` | ±4G | Accelerometer range |
| `MPU_GYRO_RANGE` | ±500°/s | Gyroscope range |
| `MPU_FILTER_BW` | 10Hz | Low-pass filter |

### 7. **Sensor Fusion**
| Constant | Value | Purpose |
|----------|-------|---------|
| `COMPLEMENTARY_FILTER_ALPHA` | 0.98 | Gyro dominance (98% gyro, 2% accel) |

### 8. **Yaw Angle Wrapping**
| Constant | Value | Purpose |
|----------|-------|---------|
| `YAW_ANGLE_MAX` | 180.0° | Max angle before wrap |
| `YAW_ANGLE_MIN` | -180.0° | Min angle before wrap |

### 9. **PID Tuning - Angle Control**
| Constant | Value | Purpose |
|----------|-------|---------|
| `kp_Roll` | 3.5 | Roll angle proportional |
| `ki_Roll` | 0.02 | Roll angle integral |
| `kd_Roll` | 1.2 | Roll angle derivative |
| `kp_Pitch` | 3.5 | Pitch angle proportional |
| `ki_Pitch` | 0.02 | Pitch angle integral |
| `kd_Pitch` | 1.2 | Pitch angle derivative |

### 10. **PID Tuning - Rate Control**
| Constant | Value | Purpose |
|----------|-------|---------|
| `PRateRoll` | 1.5 | Roll rate proportional |
| `IRateRoll` | 0.01 | Roll rate integral |
| `DRateRoll` | 0.08 | Roll rate derivative |
| `PRatePitch` | 1.5 | Pitch rate proportional |
| `IRatePitch` | 0.01 | Pitch rate integral |
| `DRatePitch` | 0.08 | Pitch rate derivative |
| `PRateYaw` | 0.5 | Yaw rate proportional |
| `IRateYaw` | 0.01 | Yaw rate integral |
| `DRateYaw` | 0.05 | Yaw rate derivative |

### 11. **PID Anti-Windup**
| Constant | Value | Purpose |
|----------|-------|---------|
| `PID_INTEGRAL_MAX` | 200.0 | Integral clamp limit |

### 12. **Failsafe Configuration**
| Constant | Value | Purpose | Source |
|----------|-------|---------|--------|
| `COMMAND_TIMEOUT` | 2000 ms | WiFi disconnect timeout | Failsafe in main.ino |
| `MAX_DT` | 0.1 s | Max allowed dt | Sensor reading in sensors.cpp |
| `MIN_DT` | 0.0001 s | Min allowed dt | Sensor reading in sensors.cpp |

### 13. **Initialization Delays**
| Constant | Value | Purpose | Source |
|----------|-------|---------|--------|
| `WIFI_INIT_DELAY` | 100 ms | WiFi mode change wait | `initializeWiFi()` in wifi_comm.cpp |
| `MPU_INIT_DELAY` | 1000 ms | MPU6050 power-up | `read_MPU6050()` in sensors.cpp |
| `SERIAL_WAIT_DELAY` | 10 ms | Serial connection wait | `read_MPU6050()` in sensors.cpp |
| `ERROR_RESTART_DELAY` | 10 ms | Error loop delay | `read_MPU6050()` in sensors.cpp |

### 14. **WiFi Configuration**
| Constant | Value | Purpose |
|----------|-------|---------|
| `WIFI_SSID` | "Esp32drone" | Access point name |
| `WIFI_PASSWORD` | "esp32drone" | WiFi password |
| `WIFI_PORT` | 80 | HTTP server port |

### 15. **Serial Communication**
| Constant | Value | Purpose |
|----------|-------|---------|
| `SERIAL_BAUD` | 115200 | Baud rate |

### 16. **UDP Configuration**
| Constant | Value | Purpose |
|----------|-------|---------|
| `udpPort` | 4210 | UDP listening port |

### 17. **Loop Timing**
| Constant | Value | Purpose |
|----------|-------|---------|
| `LOOP_INTERVAL` | 2000 µs | 500Hz loop interval |

---

## Hardcoded Values Replaced

### In `initializeWiFi()` (wifi_comm.cpp):
- ✅ `delay(100)` → `delay(WIFI_INIT_DELAY)`

### In `read_MPU6050()` (sensors.cpp):
- ✅ `delay(10)` → `delay(SERIAL_WAIT_DELAY)`
- ✅ `I2CBus.setClock(100000)` → `I2CBus.setClock(I2C_CLOCK_SPEED)`
- ✅ `delay(1000)` → `delay(MPU_INIT_DELAY)`
- ✅ `while(1) delay(10)` → `while(1) delay(ERROR_RESTART_DELAY)`

### In `readSensors()` (sensors.cpp):
- ✅ `/ 9.81` → `/ GRAVITY_CONSTANT` (accelerometer normalization, 2 places)
- ✅ `* 180/PI` → Uses math constants (gyro conversion, 3 places)
- ✅ `dt > 0.1` → `dt > MAX_DT`
- ✅ `yaw > 180` → `yaw > YAW_ANGLE_MAX`
- ✅ `yaw < -180` → `yaw < YAW_ANGLE_MIN`

### In `computePID()` (pid.cpp):
- ✅ `constrain(..., -1000, 1000)` → `constrain(..., PWM_OUTPUT_MIN, PWM_OUTPUT_MAX)` (3 places)
- ✅ `mapFloat(..., 0, 1023)` → `mapFloat(..., PWM_MOTOR_OUTPUT_MIN, PWM_MOTOR_OUTPUT_MAX)` (4 places)

### In `calibrateMPU()` (sensors.cpp):
- ✅ `accZ - 9.81` → `accZ - GRAVITY_CONSTANT`
- ✅ `delay(5)` → `delay(CALIBRATION_DELAY_MS)`

### In Command Functions (receiver.cpp):
- ✅ `goForward()`: `MAX_ATTITUDE_ANGLE`
- ✅ `goBack()`: `-MAX_ATTITUDE_ANGLE`
- ✅ `goLeft()`: `-MAX_ATTITUDE_ANGLE`
- ✅ `goRight()`: `MAX_ATTITUDE_ANGLE`
- ✅ `rotateRight()`: `MAX_ATTITUDE_ANGLE`
- ✅ `rotateLeft()`: `-MAX_ATTITUDE_ANGLE`

### In `setupMotors()` (motors.cpp):
- ✅ Motor pin assignments use `MOTOR_PINS[4]`
- ✅ PWM setup uses `PWM_FREQUENCY`, `PWM_RESOLUTION`

### In `main.ino`:
- ✅ Loop interval uses `LOOP_INTERVAL`
- ✅ UDP port uses `udpPort`

---

## Benefits of This Refactoring

✅ **Maintainability**: All magic numbers in one place  
✅ **Readability**: Descriptive constant names + comprehensive comments  
✅ **Tuning**: Easy to adjust performance by changing config.h  
✅ **Documentation**: Every constant has a clear purpose comment  
✅ **Safety**: Consistent values across all functions  
✅ **Debugging**: Easier to trace constants when troubleshooting  
✅ **Modularity**: Constants centralized, code split into logical modules  

---

## How to Modify Constants

### To Change PID Gains:
```cpp
// In config.h
const float kp_Roll = 3.5;  // Change this value
```

### To Adjust Motor Speeds:
```cpp
// In config.h
const int MOTOR_MAX = 2000;  // Increase for faster motors
```

### To Modify Failsafe Timeout:
```cpp
// In config.h
const unsigned long COMMAND_TIMEOUT = 2000;  // Change to 3000 for 3 seconds
```

### To Change I2C Speed:
```cpp
// In config.h
const int I2C_CLOCK_SPEED = 100000;  // Can be 100000 (100kHz) or 400000 (400kHz)
```

---

## Verification

✅ **All hardcoded constants removed**  
✅ **All constants moved to config.h with comments**  
✅ **Code modularized into separate files**  
✅ **No functionality changed, only organization improved**  
✅ **Constants usage updated to reflect new file locations**  

---

*Audit Updated: Post-Modularization*
