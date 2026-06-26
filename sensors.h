#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "config.h"
 
extern MPU6050 mpu;

struct SensorState {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
};

extern SensorState sensors;
extern TwoWire I2CBus;

void read_MPU6050();
void calibrateMPU();
float readSensors(float &dt);
void runMPUTests();

#endif
