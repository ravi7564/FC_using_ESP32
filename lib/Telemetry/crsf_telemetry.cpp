#include "crsf_telemetry.h"
#include "crsf.h"
#include "crsf_protocol.h"
#include <Arduino.h>
#include <string.h>

// Import CRSF serial port instantiated in main.cpp
extern HardwareSerial crsfSerial;

// Link CRC calculation function from crsf.cpp
extern uint8_t crsfCalculateCRC(const uint8_t* data, uint8_t len);

static void sendCrsfFrame(uint8_t frameType, const uint8_t *payload, uint8_t payloadLength) {
    uint8_t frame[64];
    uint8_t totalLength = payloadLength + 2;

    if (totalLength + 2 > sizeof(frame)) return;

    frame[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
    frame[1] = totalLength;
    frame[2] = frameType;

    memcpy(&frame[3], payload, payloadLength);

    uint8_t crc = crsfCalculateCRC(&frame[2], payloadLength + 1);
    frame[3 + payloadLength] = crc;

    // Transmit telemetry frame back to receiver (via crsfSerial TX)
    crsfSerial.write(frame, totalLength + 2);
}

void crsfTelemetryInit(void) {
    // Initialization hook for future extensions
}

// Sends Battery Voltage and Barometric Altitude telemetry frames
void crsfSendSensorTelemetry(float voltage, float altitude_m) {

    // ==========================================
    // 1. BATTERY FRAME (Voltage)
    // ==========================================
    uint8_t battPayload[8] = {0}; // Remaining fields (Current, mAh) initialized to zero

    uint16_t voltage_decivolts = (uint16_t)(voltage * 10.0f);
    battPayload[0] = (voltage_decivolts >> 8) & 0xFF;
    battPayload[1] = voltage_decivolts & 0xFF;

    sendCrsfFrame(CRSF_FRAMETYPE_BATTERY_SENSOR, battPayload, 8);


    // ==========================================
    // 2. GPS FRAME (Altitude)
    // ==========================================
    uint8_t gpsPayload[15] = {0}; // Remaining fields (Lat, Lon, Sats) initialized to zero

    // CRSF specification: altitude = (meters + 1000)
    uint16_t alt_encoded = (uint16_t)(altitude_m + 1000.0f);

    gpsPayload[12] = (alt_encoded >> 8) & 0xFF;
    gpsPayload[13] = alt_encoded & 0xFF;

    sendCrsfFrame(CRSF_FRAMETYPE_GPS, gpsPayload, 15);
}

// Flight Mode Frame
void crsfSendFlightMode(const char *mode) {
    uint8_t len = strlen(mode);
    if (len > 15) len = 15; // CRSF max payload size limit

    uint8_t payload[16];
    memcpy(payload, mode, len);
    payload[len] = '\0'; // Ensure null terminator

    sendCrsfFrame(CRSF_FRAMETYPE_FLIGHT_MODE, payload, len + 1);
}