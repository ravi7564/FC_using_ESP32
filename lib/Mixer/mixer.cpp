#include "mixer.h"
#include "config.h"

Mixer::Mixer()
{
}

void Mixer::begin()
{
}

MixerOutputs Mixer::mix(
    float throttle_pwm,
    float roll_correction,
    float pitch_correction,
    float yaw_correction)
{
    MixerOutputs out = {
        PWM_CENTER,
        PWM_CENTER,
        PWM_CENTER,
        PWM_MIN
    };

    // --------------------------------------------------
    // Throttle
    // --------------------------------------------------

    out.throttle_us = constrain(
        (int)throttle_pwm,
        PWM_MIN,
        PWM_MAX
    );

#if ACTIVE_WING_TYPE == WING_TYPE_DELTA

    // --------------------------------------------------
    // Delta Wing - Elevon Mixing
    // --------------------------------------------------

    int rawLeft =
        PWM_CENTER +
        (int)pitch_correction +
        (int)roll_correction;

    int rawRight =
        PWM_CENTER +
        (int)pitch_correction -
        (int)roll_correction;

    out.aileron_us = constrain(
        rawLeft,
        PWM_MIN,
        PWM_MAX
    );

    out.elevator_us = constrain(
        rawRight,
        PWM_MIN,
        PWM_MAX
    );

    out.rudder_us = constrain(
        PWM_CENTER + (int)yaw_correction,
        PWM_MIN,
        PWM_MAX
    );

#else

    // --------------------------------------------------
    // Standard Fixed Wing
    // --------------------------------------------------

    out.aileron_us = constrain(
        PWM_CENTER + (int)roll_correction,
        PWM_MIN,
        PWM_MAX
    );

    out.elevator_us = constrain(
        PWM_CENTER - (int)pitch_correction,
        PWM_MIN,
        PWM_MAX
    );

    out.rudder_us = constrain(
        PWM_CENTER + (int)yaw_correction,
        PWM_MIN,
        PWM_MAX
    );

#endif

    return out;
}