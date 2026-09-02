#include "gyro.h"
#include "config.h"

Gyroscope::Gyroscope() {
    _lastReadUs = 0;
    _intervalUs = IMU_INTERVAL_US;  // 100Hz from config
    _gyroData = {0, 0, 0, false};
    _offX = _offY = _offZ = 0;
}

bool Gyroscope::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    delay(100);

    // Verify sensor is connected (WHO_AM_I check)
    uint8_t whoAmI = readReg(REG_WHO_AM_I);
    if (whoAmI != WHO_AM_I_EXPECTED) {
        return false;  // MPU6050/6500 not found or wrong chip
    }

    // Wake up from sleep mode, use internal oscillator
    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(50);

    // Sample Rate Divider: 8kHz / (1 + 7) = 1kHz internal, then apply LPF
    writeReg(REG_SMPLRT_DIV, 0x07);

    // Digital Low Pass Filter: BW=42Hz, delay=4.8ms
    writeReg(REG_CONFIG, 0x03);

    // Gyroscope Config: ±2000°/s full scale
    writeReg(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE);

    // Ensure Accelerometer Config is maintained at ±8G (prevents ±2G scale corruption)
    writeReg(REG_ACCEL_CONFIG, 0x10);

    // Enable interrupts (optional, for data ready)
    writeReg(REG_INT_ENABLE, 0x01);  // Data ready interrupt

    delay(50);
    _lastReadUs = micros();
    return true;
}

void Gyroscope::setCalibrationOffsets(int16_t offX, int16_t offY, int16_t offZ) {
    _offX = offX;
    _offY = offY;
    _offZ = offZ;
}

bool Gyroscope::dataReady() {
    uint32_t now = micros();
    return (now - _lastReadUs >= _intervalUs);
}

bool Gyroscope::update() {
    uint8_t buffer[BURST_LEN] = {0};
    readBytes(REG_GYRO_XOUT_H, buffer, BURST_LEN);

    // Sanity check: if all zeros or all 0xFF, data is invalid
    if ((buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0) ||
        (buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFF)) {
        _gyroData.valid = false;
        return false;
    }

    // Extract gyroscope data (6 bytes: X, Y, Z)
    int16_t rawGx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawGy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawGz = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Apply calibration offsets
    _gyroData.gx = rawGx - _offX;
    _gyroData.gy = rawGy - _offY;
    _gyroData.gz = rawGz - _offZ;
    _gyroData.valid = true;

    _lastReadUs = micros();
    return true;
}

void Gyroscope::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t Gyroscope::readReg(uint8_t reg) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 0;  // I2C error
    }

    Wire.requestFrom(ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void Gyroscope::readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return;  // I2C error, buffer remains zeroed
    }

    uint8_t received = Wire.requestFrom(ADDR, len);
    if (received != len) {
        return;  // Not enough bytes received
    }

    uint8_t i = 0;
    while (Wire.available() && i < len) {
        buffer[i++] = Wire.read();
    }
}