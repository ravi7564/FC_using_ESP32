#ifndef MIXER_H
#define MIXER_H

#include <Arduino.h>
#include "config.h"

struct MixerOutputs
{
    int aileron_us;
    int elevator_us;
    int rudder_us;
    int throttle_us;
};

class Mixer
{
public:
    Mixer();

    void begin();

    MixerOutputs mix(
        float throttle_pwm,
        float roll_correction,
        float pitch_correction,
        float yaw_correction
    );
};

#endif