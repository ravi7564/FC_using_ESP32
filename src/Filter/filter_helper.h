#ifndef FILTER_HELPER_H
#define FILTER_HELPER_H

#include <Arduino.h>

void initFilter(float sampleRateHz);

// Rapidly converges filter attitude during boot sequence
void warmupFilter(float ax_g, float ay_g, float az_g, float mx, float my, float mz, bool mag_valid);

void updateFilter(float gx_dps, float gy_dps, float gz_dps,
                  float ax_g, float ay_g, float az_g,
                  float mx, float my, float mz, bool mag_valid);

// Clean attitude angles (degrees) for flight control
float getRoll();
float getPitch();
float getYaw(); // Returns yaw - yaw_offset (corrected heading)
float getRawYaw(); // Returns raw yaw without offset (used during calibration)

// Set yaw offset (called once during boot calibration)
void setYawOffset(float offset);

#endif