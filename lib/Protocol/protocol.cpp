#include "protocol.h"
#include <Arduino.h>

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
    float yaw)
{
    TelemetryPayload payload;
    payload.mode = mode;
    payload.arm = arm;
    payload.throttle = throttle;
    payload.aileron = aileron;
    payload.elevator = elevator;
    payload.rudder = rudder;
    payload.altitude = altitude;
    payload.roll = roll;
    payload.pitch = pitch;
    payload.yaw = yaw;

    const uint8_t header1 = 0x24; // '$'
    const uint8_t header2 = 0x4D; // 'M'
    const uint8_t msgId   = 0x01; // Telemetry ID
    const uint8_t length  = sizeof(TelemetryPayload); // 26 bytes

    // Compute XOR Checksum over msgId, length, and all payload bytes
    uint8_t checksum = msgId ^ length;
    const uint8_t *payloadBytes = (const uint8_t *)&payload;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= payloadBytes[i];
    }

    // Build atomic 31-byte frame buffer
    uint8_t frame[31];
    frame[0] = header1;
    frame[1] = header2;
    frame[2] = msgId;
    frame[3] = length;
    memcpy(&frame[4], payloadBytes, length);
    frame[4 + length] = checksum;

    // Single atomic write to UART
    Serial.write(frame, sizeof(frame));
}
