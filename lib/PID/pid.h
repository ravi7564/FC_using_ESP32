#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float previousInput;
    float dFiltered;
    bool isFirstRun;
    float outMin, outMax;
    float maxI;

public:
    PIDController(float p, float i, float d, float maxIntegral, float minOut, float maxOut);
    void reset();
    float compute(float setpoint, float input, float dt);
};

class FlightControlSystem {
private:
    PIDController rollPID;
    PIDController pitchPID;
    PIDController yawPID;

    float targetYawAngle;
    bool yawInitialized;

public:
    FlightControlSystem();
    void reset();

    // Main flight control computation function
    void computeControl(
        float stickRoll, float stickPitch, float stickYaw,
        float currentAngleRoll, float currentAnglePitch, float currentAngleYaw,
        float dt,
        float &outRoll, float &outPitch, float &outYaw
    );
};

#endif