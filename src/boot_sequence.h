#ifndef BOOT_SEQUENCE_H
#define BOOT_SEQUENCE_H

#include <Arduino.h>
#include "Sensors/Magnetometer/magnetometer.h"
#include "Sensors/Accel/accel.h"
#include "Sensors/Gyro/gyro.h"
#include "Sensors/Barometer/barometer.h"
#include <pid.h>

// Primary boot sequence orchestrating sensor init, calibration, and filter alignment
void runBootSequence(Accelerometer &accel, Gyroscope &gyro, Barometer &baro, Magnetometer &mag, FlightControlSystem &fc);

#endif