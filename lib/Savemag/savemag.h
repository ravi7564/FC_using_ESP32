#ifndef SAVEMAG_H
#define SAVEMAG_H

#include <Arduino.h>
#include <Wire.h>
#include "../../src/Sensors/Magnetometer/magnetometer.h"

// ============= NVS Storage Functions =============
// Save Magnetometer hard-iron offsets to NVS
bool saveMagToNVS(int16_t xOff, int16_t yOff, int16_t zOff);

// Load Magnetometer hard-iron offsets from NVS
bool loadMagFromNVS(int16_t &xOff, int16_t &yOff, int16_t &zOff);

// ============= Calibration & Save Routine =============
// Run figure-8 calibration sweep, calculate hard-iron offsets, save to NVS, and apply to sensor
bool calibrateAndSaveMag(Magnetometer &mag, uint32_t durationMs = 10000);

#endif // SAVEMAG_H
