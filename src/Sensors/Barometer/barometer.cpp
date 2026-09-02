#include "barometer.h"
#include "config.h"

Barometer::Barometer() {
    _addr = ADDR_PRIM;
    _seaLevelPressure_pa = SEA_LEVEL_PRESSURE_PA;  // From config.h
    _pressureOffsetPa = 0.0f;
    _lastReadUs = 0;
    _intervalUs = BARO_INTERVAL_US;                // From config.h
    _data = {false, 0.0f, 0.0f, 0.0f};
    t_fine = 0;
}

bool Barometer::begin(uint8_t i2c_addr) {
    _addr = i2c_addr;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);  // From config.h
    delay(10);

    // Soft reset
    writeRegister(0xE0, 0xB6);
    delay(100);

    // Verify chip ID
    uint8_t chipId = 0;
    if (!readRegisters(REG_CHIP_ID, &chipId, 1)) {
        return false;
    }
    if (chipId != 0x58 && chipId != 0x59 && chipId != 0x60) {
        return false;  // Unknown BMP chip variant
    }

    // Configuration: normal mode, oversampling
    writeRegister(REG_CONTROL, 0x57);   // Oversampling x4 for all, normal mode
    writeRegister(REG_CONFIG, 0x10);    // IIR filter coefficient 16

    delay(50);

    // Load calibration data from sensor
    readCalibrationData();

    _lastReadUs = micros();
    return dig_P1 != 0;  // Verify calibration data was loaded
}

void Barometer::setSeaLevelPressure(float pressure_hpa) {
    _seaLevelPressure_pa = pressure_hpa * 100.0f;
}

void Barometer::setCalibrationOffset(float pressureOffsetPa) {
    _pressureOffsetPa = pressureOffsetPa;
}

bool Barometer::update() {
    uint32_t now = micros();
    if (now - _lastReadUs < _intervalUs) {
        return false;  // Not enough time has passed
    }
    _lastReadUs = now;

    uint8_t buffer[6] = {0};
    if (!readRegisters(REG_PRESSURE, buffer, 6)) {
        _data.valid = false;
        return false;
    }

    // Extract pressure ADC (20-bit)
    int32_t adc_P = ((((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2])) >> 4;

    // Extract temperature ADC (20-bit)
    int32_t adc_T = ((((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | (uint32_t)buffer[5])) >> 4;

    // Run compensation algorithms
    int32_t temp_calibrated = compensateTemperature(adc_T);
    uint32_t press_calibrated_q24 = compensatePressure(adc_P);

    // Divide by 256.0f because Bosch 64-bit algorithm returns pressure in Q24.8 format
    float pressurePa = ((float)press_calibrated_q24 / 256.0f) + _pressureOffsetPa;

    // Sanity checks
    if (!isfinite(pressurePa) || pressurePa < 30000.0f || pressurePa > 200000.0f) {
        _data.valid = false;
        return false;
    }

    if (_seaLevelPressure_pa <= 0.0f) {
        _data.valid = false;
        return false;
    }

    float tempC = (float)temp_calibrated / 100.0f;

    // Hypsometric formula
    float altitudeM = 44330.0f * (1.0f - powf(pressurePa / _seaLevelPressure_pa, 0.1903f));

    _data.pressure_pa = pressurePa;
    _data.temperature_c = tempC;
    _data.altitude_m = altitudeM;
    _data.valid = true;

    return true;
}

void Barometer::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool Barometer::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;  // I2C transmission error
    }

    uint8_t received = Wire.requestFrom(_addr, length);
    if (received != length) {
        return false;  // Didn't receive expected number of bytes
    }

    uint8_t i = 0;
    while (Wire.available() && i < length) {
        buffer[i++] = Wire.read();
    }

    return (i == length);  // Verify we read all expected bytes
}

void Barometer::readCalibrationData() {
    uint8_t calib[24] = {0};
    if (!readRegisters(0x88, calib, 24)) {
        dig_P1 = 0;  // Mark as invalid
        return;
    }

    // Temperature calibration
    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

    // Pressure calibration
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);
}

int32_t Barometer::compensateTemperature(int32_t adc_T) {
    int32_t var1, var2;
    var1 = (((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

uint32_t Barometer::compensatePressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 += (var1 * (int64_t)dig_P5) << 17;
    var2 += ((int64_t)dig_P4) << 35;

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0;  // Avoid division by zero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}