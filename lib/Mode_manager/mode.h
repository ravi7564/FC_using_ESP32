#ifndef MODE_H
#define MODE_H

#include <stdint.h>

enum FlightMode {
    MODE_STABILIZATION,
    MODE_MANUAL,
    MODE_HORIZON_LOCK
};

class ModeManager {
public:
    ModeManager();
    void update(uint16_t modeChannelPwm);
    FlightMode getMode() const;

private:
    FlightMode currentMode;
};

#endif