#ifndef CRSF_TELEMETRY_H
#define CRSF_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

// Initialize CRSF telemetry module
void crsfTelemetryInit();

// voltage: Battery voltage (example 12.6)
// altitude_m: Barometer height in meters
void crsfSendSensorTelemetry(float voltage, float altitude_m);

// Transmit flight mode string
void crsfSendFlightMode(const char *mode);

#endif