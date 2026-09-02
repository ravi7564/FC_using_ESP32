#include "mode.h"

ModeManager::ModeManager()
    : currentMode(MODE_STABILIZATION)
{
}

void ModeManager::update(uint16_t modeChannelPwm)
{
    // SAFETY CHECK: If PWM value is invalid (e.g. 0, or outside 900-2100us),
    // default to safe STABILIZATION mode.
    if (modeChannelPwm < 900 || modeChannelPwm > 2100)
    {
        currentMode = MODE_STABILIZATION;
    }
    // Switch Position 1 (Down)
    else if (modeChannelPwm < 1300)
    {
        currentMode = MODE_STABILIZATION;
    }
    // Switch Position 2 (Center)
    else if (modeChannelPwm < 1700)
    {
        currentMode = MODE_MANUAL;
    }
    // Switch Position 3 (Up)
    else
    {
        currentMode = MODE_HORIZON_LOCK;
    }
}

FlightMode ModeManager::getMode() const
{
    return currentMode;
}