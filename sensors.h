#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "config.h"

extern MPU6050 mpu;
extern float accErrorX, accErrorY, accErrorZ;
extern float gyroErrorX, gyroErrorY, gyroErrorZ;
extern float roll, pitch;
extern float gyroX, gyroY, gyroZ;
extern TwoWire I2CBus;

void read_MPU6050();
void calibrateMPU();
float readSensors();

#endif
