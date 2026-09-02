#include "actuators.h"
#include "config.h"

Actuators::Actuators() {
    _channelCount = 0;
    _maxDuty = (1UL << 14) - 1;
    _pwmPeriodUs = 20000;
    for (int i = 0; i < 16; i++) {
        _lastOutput[i] = (i == OUT_MOTOR) ? PWM_MIN : PWM_CENTER;
    }
}

void Actuators::begin(uint8_t channelCount, const int* pins, int pwmFreq, int pwmResolutionBits) {
    _channelCount = (channelCount < 16) ? channelCount : 16;
    _maxDuty = (1UL << pwmResolutionBits) - 1;
    _pwmPeriodUs = (pwmFreq > 0) ? (1000000UL / pwmFreq) : 20000;

    for (uint8_t i = 0; i < _channelCount; i++) {
        ledcSetup(i, pwmFreq, pwmResolutionBits);
        ledcAttachPin(pins[i], i);
        if (i == OUT_MOTOR) {
            write(i, PWM_MIN);
        } else {
            write(i, PWM_CENTER);
        }
    }
}

void Actuators::write(uint8_t channelIndex, int microseconds) {
    if (channelIndex >= _channelCount) return;

    if (microseconds < MIN_US) microseconds = MIN_US;
    if (microseconds > MAX_US) microseconds = MAX_US;

    _lastOutput[channelIndex] = microseconds;

    uint32_t duty = ((uint32_t)microseconds * _maxDuty) / _pwmPeriodUs;
    ledcWrite(channelIndex, duty);
}

int Actuators::getLastOutput(uint8_t channelIndex) {
    if (channelIndex >= _channelCount) return PWM_CENTER;
    return _lastOutput[channelIndex];
}

void Actuators::updateFromMixer(const int* mixedOutputs, uint8_t count) {
    uint8_t limit = (count < _channelCount) ? count : _channelCount;
    for (uint8_t i = 0; i < limit; i++) {
        write(i, mixedOutputs[i]);
    }
}