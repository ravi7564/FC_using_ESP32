#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>


#define BARO_SAMPLE_RATE_HZ     50       // 50Hz update rate
#define BARO_INTERVAL_US        20000    // 1000000 / 50 = 20000 microseconds

struct BaroData {
    bool valid;
    float pressure_pa;
    float temperature_c;
    float altitude_m;
};

class Barometer {
public:
    static constexpr uint8_t ADDR_PRIM = 0x76;
    static constexpr uint8_t ADDR_SEC  = 0x77;

    Barometer();
    bool begin(uint8_t i2c_addr = ADDR_PRIM);
    bool update();
    BaroData getData() const { return _data; }
    void setSeaLevelPressure(float pressure_hpa);
    void setCalibrationOffset(float pressureOffsetPa);

private:
    static constexpr uint8_t REG_DIG_T1   = 0x88;
    static constexpr uint8_t REG_DIG_P1   = 0x8E;
    static constexpr uint8_t REG_CHIP_ID  = 0xD0;
    static constexpr uint8_t REG_CONTROL  = 0xF4;
    static constexpr uint8_t REG_CONFIG   = 0xF5;
    static constexpr uint8_t REG_PRESSURE = 0xF7;

    uint8_t _addr;
    BaroData _data;
    float _seaLevelPressure_pa;
    float _pressureOffsetPa;

    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine;

    uint32_t _lastReadUs;
    uint32_t _intervalUs;

    void readCalibrationData();
    void writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);

    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
};

#endif