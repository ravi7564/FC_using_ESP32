#include "magnetometer.h"
#include "config.h"

Magnetometer::Magnetometer() {
    _lastReadUs = 0;
    _intervalUs = MAG_INTERVAL_US;
    _data = {0, 0, 0, 0, false};
    _offsetX = 0;
    _offsetY = 0;
    _offsetZ = 0;
}

bool Magnetometer::begin() {
    // 1. Standby mode
    writeRegister(REG_CONFIG, 0x00);
    delay(10);

    // 2. Set Pulse
    writeRegister(REG_CONFIG, 0x01);
    delay(20);

    // 3. Mode configuration
    writeRegister(REG_MODE, 0x05);

    _lastReadUs = micros();
    return true;
}

void Magnetometer::setCalibrationOffsets(int16_t xOffset, int16_t yOffset, int16_t zOffset) {
    _offsetX = xOffset;
    _offsetY = yOffset;
    _offsetZ = zOffset;
}

bool Magnetometer::update() {
    uint32_t now = micros();
    if (now - _lastReadUs < _intervalUs) {
        return false;
    }
    _lastReadUs = now;

    uint8_t status = readRegister(REG_STATUS);
    uint8_t buffer[BURST_LEN] = {0};
    readBytes(REG_DATA_OUT_X_LSB, buffer, BURST_LEN);

    int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Glitch filter
    if ((rawX == 0 && rawY == 0 && rawZ == 0) || (rawX == rawY && rawY == rawZ)) {
        _data.valid = false;
        return false;
    }

    // Apply offsets
    _data.x = rawX - _offsetX;
    _data.y = rawY - _offsetY;
    _data.z = rawZ - _offsetZ;
    _data.status = status;
    _data.valid = true;

    return true;
}

void Magnetometer::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MAG_SENSOR_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t Magnetometer::readRegister(uint8_t reg) {
    Wire.beginTransmission(MAG_SENSOR_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)MAG_SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void Magnetometer::readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {
    Wire.beginTransmission(MAG_SENSOR_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)MAG_SENSOR_ADDR, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) {
        buffer[i++] = Wire.read();
    }
}