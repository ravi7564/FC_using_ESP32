#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>

class Actuators {
public:
    Actuators();
    void begin(uint8_t channelCount, const int* pins, int pwmFreq, int pwmResolutionBits);
    void write(uint8_t channelIndex, int microseconds);
    int getLastOutput(uint8_t channelIndex);

    // Direct interface method to ingest mixed pulse-width values from the flight control mixer pipeline
    void updateFromMixer(const int* mixedOutputs, uint8_t count);

private:
    uint8_t _channelCount;
    int _lastOutput[16];
    uint32_t _maxDuty;
    uint32_t _pwmPeriodUs;

    static constexpr int MIN_US = 900;
    static constexpr int MAX_US = 2100;
};

#endif