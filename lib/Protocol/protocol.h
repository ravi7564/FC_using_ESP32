#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// 1. Binary Telemetry Payload Structure (Packed 26 bytes)
#pragma pack(push, 1)
struct TelemetryPayload {
    uint8_t mode;      // 1 byte (0: STBL, 1: MANUAL, 2: HORIZON)
    uint8_t arm;       // 1 byte (0: DISARMED, 1: ARMED, 2: FAILSAFE)
    int16_t throttle;  // 2 bytes (1000 - 2000 us)
    int16_t aileron;   // 2 bytes (1000 - 2000 us)
    int16_t elevator;  // 2 bytes (1000 - 2000 us)
    int16_t rudder;    // 2 bytes (1000 - 2000 us)
    float altitude;    // 4 bytes (meters)
    float roll;        // 4 bytes (degrees)
    float pitch;       // 4 bytes (degrees)
    float yaw;         // 4 bytes (degrees)
};
#pragma pack(pop)

// Function to broadcast binary telemetry frame over Serial
void sendBinaryTelemetry(
    uint8_t mode,
    uint8_t arm,
    int16_t throttle,
    int16_t aileron,
    int16_t elevator,
    int16_t rudder,
    float altitude,
    float roll,
    float pitch,
    float yaw
);

#endif
