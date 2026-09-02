#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============= MAGNETOMETER CONFIGURATION (Hardcoded) =============
#define MAG_SENSOR_ADDR         0x2C
#define MAG_SAMPLE_RATE_HZ      100      // 100Hz update rate
#define MAG_INTERVAL_US         10000    // 1000000 / 100 = 10000 microseconds

struct MagData {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t status;
    bool valid;
};

class Magnetometer {
public:
    Magnetometer();
    bool begin();
    bool update();
    MagData getData() const { return _data; }
    void setCalibrationOffsets(int16_t xOffset, int16_t yOffset, int16_t zOffset);

private:
    static constexpr uint8_t REG_DATA_OUT_X_LSB = 0x00;
    static constexpr uint8_t REG_STATUS         = 0x06;
    static constexpr uint8_t REG_MODE           = 0x09;
    static constexpr uint8_t REG_CONFIG         = 0x0A;
    static constexpr uint8_t BURST_LEN          = 6;

    MagData _data;
    uint32_t _lastReadUs;
    uint32_t _intervalUs;
    int16_t _offsetX, _offsetY, _offsetZ;

    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len);
};

#endif