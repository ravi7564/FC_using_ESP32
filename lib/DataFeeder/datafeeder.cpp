#include "datafeeder.h"
#include "config.h"
#include <cstring>
#include <Arduino.h>

template<typename T>
static T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

DataFeeder::DataFeeder() {
    for (uint8_t i = 0; i < 16; i++) {
        pwm_data.channels[i] = PWM_CENTER;
    }
    pwm_data.channels[RC_CH3_MOTOR] = PWM_MIN;       // CH3 Motor at 1000us
    pwm_data.channels[RC_CH5_ARM] = PWM_MIN;         // CH5 Disarmed at 1000us
    pwm_data.channels[RC_CH6_FLIGHTMODE] = PWM_MIN;  // CH6 Flight mode at 1000us
    pwm_data.valid = false;
    pwm_data.timestamp_us = 0;
    last_data_time_us = 0;
}

uint16_t DataFeeder::crsfToPwm(uint16_t crsf_val) {
    crsf_val = clamp(crsf_val, (uint16_t)CRSF_MIN, (uint16_t)CRSF_MAX);
    uint16_t pwm = ((uint32_t)(crsf_val - CRSF_MIN) * PWM_RANGE) / CRSF_RANGE + PWM_MIN;
    return clamp(pwm, (uint16_t)PWM_MIN, (uint16_t)PWM_MAX);
}

void DataFeeder::feedCRSFData(bool is_valid, const uint16_t crsf_channels[16]) {
    if (!is_valid) {
        return;
    }

    for (uint8_t i = 0; i < 16; i++) {
        pwm_data.channels[i] = crsfToPwm(crsf_channels[i]);
    }

    uint32_t now = micros();
    pwm_data.timestamp_us = now;
    last_data_time_us = now;
    pwm_data.valid = true;
}

uint16_t DataFeeder::getChannelPWM(uint8_t channel) {
    if (channel >= 16) return PWM_CENTER;
    return pwm_data.channels[channel];
}

bool DataFeeder::isValid() {
    if (last_data_time_us == 0 || (micros() - last_data_time_us) > DATA_TIMEOUT_US) {
        pwm_data.valid = false;
        return false;
    }
    return pwm_data.valid;
}

uint32_t DataFeeder::getTimestamp() {
    return pwm_data.timestamp_us;
}