#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>
#include <Wire.h>

// ============= ACCELEROMETER CONFIGURATION (Hardcoded) =============
#define ACCEL_RANGE_G           8        // ±8G full scale
#define ACCEL_LSB_PER_G         4096     // For ±8G: 4096 LSB/G
#define ACCEL_CONFIG_VALUE      0x10     // REG_ACCEL_CONFIG register value for ±8G

struct AccelData {
    int16_t x, y, z;
    bool valid;
};

class Accelerometer {
public:
    Accelerometer();
    bool begin();
    bool update();
    AccelData getData() const { return _accelData; }
    bool dataReady();

    void setCalibrationOffsets(int16_t offX, int16_t offY, int16_t offZ);

private:
    static constexpr uint8_t ADDR             = 0x68;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_ENABLE   = 0x38;
    static constexpr uint8_t REG_INT_STATUS   = 0x3A;
    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;
    static constexpr uint8_t BURST_LEN         = 6;

    AccelData _accelData;
    int16_t _offX, _offY, _offZ;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len);
};

#endif