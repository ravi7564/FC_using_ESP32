#include "filter_helper.h"
#include <MadgwickAHRS.h>
#include <math.h>

Madgwick filter;
float yaw_offset = 0.0f; // Calibrated heading offset (loaded from NVS at boot)

void initFilter(float sampleRateHz) {
    filter.begin(sampleRateHz);
}

// Fast-forward initial filter convergence
void warmupFilter(float ax_g, float ay_g, float az_g, float mx, float my, float mz, bool mag_valid) {
    // Iterate 500 times with zero gyro rates to settle orientation
    for(int i = 0; i < 500; i++) {
        updateFilter(0.0f, 0.0f, 0.0f, ax_g, ay_g, az_g, mx, my, mz, mag_valid);
    }
}

void updateFilter(float gx_dps, float gy_dps, float gz_dps,
                  float ax_g, float ay_g, float az_g,
                  float mx, float my, float mz, bool mag_valid) {

    if (mag_valid) {
        float aligned_mx = my;
        float aligned_my = mx;
        float aligned_mz = -mz;
        filter.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, aligned_mx, aligned_my, aligned_mz);
    } else {
        filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
    }
}

// ---- Clean Getters ----
float getRoll() {
    return filter.getRoll();
}

float getPitch() {
    return -filter.getPitch(); // Inverted for True Horizon
}

float getYaw() {
    float rawYaw = -filter.getYaw(); // Inverted so Left turn decreases yaw (-) and Right turn increases yaw (+)
    float correctedYaw = rawYaw - yaw_offset;

    // Wrap to [-180, 180]
    while (correctedYaw > 180.0f) correctedYaw -= 360.0f;
    while (correctedYaw < -180.0f) correctedYaw += 360.0f;

    return correctedYaw;
}

float getRawYaw() {
    // Return raw yaw without offset (used during calibration)
    return -filter.getYaw();
}

void setYawOffset(float offset) {
    yaw_offset = offset;
    Serial.printf("[FILTER] Yaw offset applied: %.2f degrees (0-reference set)\n", offset);
}