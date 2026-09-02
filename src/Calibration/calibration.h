#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "Sensors/Magnetometer/magnetometer.h"
#include "Sensors/Accel/accel.h"
#include "Sensors/Gyro/gyro.h"
#include "Sensors/Barometer/barometer.h"

void initAndLoadMagnetometer(Magnetometer &mag);
bool calibrateMagnetometer(Magnetometer &mag, uint32_t durationMs = 10000);
void performSensorCalibration(Accelerometer &accel, Gyroscope &gyro, Barometer &baro);

#endif