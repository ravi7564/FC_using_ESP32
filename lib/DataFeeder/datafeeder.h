#ifndef DATAFEEDER_H
#define DATAFEEDER_H

#include <cstdint>
#include "config.h"

#define CRSF_MIN          172
#define CRSF_CENTER       992
#define CRSF_MAX          1811
#define CRSF_RANGE        (CRSF_MAX - CRSF_MIN)
#define PWM_RANGE         (PWM_MAX - PWM_MIN)
#define DATA_TIMEOUT_US   300000   // 300ms timeout for responsive failsafe

class DataFeeder {
private:
    struct {
        uint16_t channels[16];
        uint32_t timestamp_us;
        bool valid;
    } pwm_data;

    uint32_t last_data_time_us;

    static uint16_t crsfToPwm(uint16_t crsf_val);

public:
    DataFeeder();

    void feedCRSFData(bool is_valid, const uint16_t crsf_channels[16]);
    uint16_t getChannelPWM(uint8_t channel);
    bool isValid();
    uint32_t getTimestamp();
};

#endif