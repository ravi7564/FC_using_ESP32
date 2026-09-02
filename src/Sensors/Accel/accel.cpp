#include "accel.h"
#include "config.h"

Accelerometer::Accelerometer() {
    _accelData = {0, 0, 0, false};
    _offX = 0;
    _offY = 0;
    _offZ = 0;
}

bool Accelerometer::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    delay(100);

    // Verify sensor is connected (WHO_AM_I check)
    uint8_t whoAmI = readReg(REG_WHO_AM_I);
    if (whoAmI != WHO_AM_I_EXPECTED) {
        return false;  // MPU6050/MPU6500 not found or wrong chip
    }

    // Reset sensor to default state
    writeReg(REG_PWR_MGMT_1, 0x80);  // Reset bit
    delay(100);

    // Wake up from sleep mode, use internal oscillator
    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(50);

    // Sample Rate Divider: 8kHz / (1 + 7) = 1kHz internal, then apply LPF
    writeReg(REG_SMPLRT_DIV, 0x07);

    // Digital Low Pass Filter: BW=42Hz, delay=4.8ms
    writeReg(REG_CONFIG, 0x03);

    // Accelerometer Config: ±8G full scale
    writeReg(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE);

    // Ensure Gyroscope Config: ±2000°/s full scale
    writeReg(REG_GYRO_CONFIG, 0x18);

    // Enable interrupts (optional, for data ready)
    writeReg(REG_INT_ENABLE, 0x01);  // Data ready interrupt

    delay(50);
    return true;
}

void Accelerometer::setCalibrationOffsets(int16_t offX, int16_t offY, int16_t offZ) {
    _offX = offX;
    _offY = offY;
    _offZ = offZ;
}

bool Accelerometer::update() {
    uint8_t buffer[BURST_LEN] = {0};
    readBytes(REG_ACCEL_XOUT_H, buffer, BURST_LEN);

    // Sanity check: if all zeros or all 0xFF, data is invalid
    if ((buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0) ||
        (buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFF)) {
        _accelData.valid = false;
        return false;
    }

    // Extract accelerometer data (6 bytes: X, Y, Z)
    int16_t rawX = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawY = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawZ = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Apply calibration offsets
    _accelData.x = rawX - _offX;
    _accelData.y = rawY - _offY;
    _accelData.z = rawZ - _offZ;
    _accelData.valid = true;

    return true;
}

void Accelerometer::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t Accelerometer::readReg(uint8_t reg) {
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

void Accelerometer::readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {
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