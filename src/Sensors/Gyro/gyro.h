#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>

// ============= GYROSCOPE CONFIGURATION (Hardcoded) =============
#define GYRO_RANGE_DPS          2000     // ±2000°/s full scale
#define GYRO_LSB_PER_DPS        16.375f  // For ±2000°/s: 16.375 LSB/°/s
#define GYRO_CONFIG_VALUE       0x18     // REG_GYRO_CONFIG register value for ±2000°/s

struct GyroData {
    int16_t gx, gy, gz;
    bool valid;
};

class Gyroscope {
public:
    Gyroscope();
    bool begin();
    bool update();
    GyroData getData() const { return _gyroData; }
    bool dataReady();

    void setCalibrationOffsets(int16_t offX, int16_t offY, int16_t offZ);

private:
    static constexpr uint8_t ADDR             = 0x68;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t REG_GYRO_XOUT_H  = 0x43;
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_ENABLE   = 0x38;
    static constexpr uint8_t REG_INT_STATUS   = 0x3A;
    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;
    static constexpr uint8_t BURST_LEN         = 6; // 6 bytes for Gyro (X, Y, Z)

    GyroData _gyroData;
    uint32_t _lastReadUs;
    uint32_t _intervalUs;

    int16_t _offX, _offY, _offZ;

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len);
};

#endif