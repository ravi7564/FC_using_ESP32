#include "pid.h"

// --- PID Controller Logic ---
PIDController::PIDController(float p, float i, float d, float maxIntegral, float minOut, float maxOut) {
    kp = p; ki = i; kd = d;
    maxI = maxIntegral; outMin = minOut; outMax = maxOut;
    reset();
}

void PIDController::reset() {
    integral = 0.0f; previousInput = 0.0f; dFiltered = 0.0f; isFirstRun = true;
}

float PIDController::compute(float setpoint, float input, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = setpoint - input;

    float pTerm = kp * error;
    integral += (error * dt);
    if (integral > maxI) integral = maxI;
    else if (integral < -maxI) integral = -maxI;
    float iTerm = ki * integral;

    float dTerm = 0.0f;
    if (!isFirstRun) {
        float rawDerivative = (input - previousInput) / dt;
        // Low-pass filter on derivative (~20Hz cutoff) to eliminate high-frequency motor vibration & servo chatter
        const float alpha = 0.3f;
        dFiltered = dFiltered + alpha * (rawDerivative - dFiltered);
        dTerm = -kd * dFiltered;
    }
    previousInput = input; isFirstRun = false;

    float output = pTerm + iTerm + dTerm;
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    return output;
}

// --- Flight Control System Logic ---
// Tuned for stable Fixed-Wing flight (prevents high-speed oscillations and hunting)
FlightControlSystem::FlightControlSystem()
    : rollPID(2.0f, 0.2f, 0.03f, 100.0f, -450.0f, 450.0f),
      pitchPID(2.2f, 0.2f, 0.03f, 100.0f, -450.0f, 450.0f),
      yawPID(1.0f, 0.05f, 0.00f, 50.0f, -300.0f, 300.0f) {
    reset();
}

void FlightControlSystem::reset() {
    rollPID.reset(); pitchPID.reset(); yawPID.reset();
    yawInitialized = false; targetYawAngle = 0.0f;
}

void FlightControlSystem::computeControl(
    float stickRoll, float stickPitch, float stickYaw,
    float currentAngleRoll, float currentAnglePitch, float currentAngleYaw,
    float dt,
    float &outRoll, float &outPitch, float &outYaw)
{
    // 1. Roll & Pitch Angle Stabilization (Gentle & stable response, ~90us deflection at 45 deg error)
    outRoll = rollPID.compute(stickRoll, currentAngleRoll, dt);
    outPitch = pitchPID.compute(stickPitch, currentAnglePitch, dt);

    // 2. Yaw Logic (Direct rudder control on stick deflection OR during banking, Heading Hold when wings level)
    const float YAW_DEADBAND = 20.0f; // in microseconds from center
    const float ROLL_COORDINATION_THRESHOLD = 5.0f; // degrees roll stick deflection

    if (abs(stickYaw) > YAW_DEADBAND || abs(stickRoll) > ROLL_COORDINATION_THRESHOLD) {
        // When pilot moves rudder stick or banks into a turn: Direct rudder authority (does not fight bank turn)
        outYaw = stickYaw;
        yawInitialized = false; // Reset lock so new heading is locked once stick returns to center and plane levels
    } else {
        // When stick is centered: Heading Hold Mode
        if (!yawInitialized) {
            targetYawAngle = currentAngleYaw; // Lock current heading as new target
            yawInitialized = true;
            yawPID.reset(); // Reset accumulated integral and derivative error
        }

        // Shortest Path Calculation (Left turn produces positive rudder correction > 1500, Right turn produces < 1500)
        float yawError = targetYawAngle - currentAngleYaw;
        if (yawError > 180.0f) yawError -= 360.0f;
        if (yawError < -180.0f) yawError += 360.0f;

        outYaw = yawPID.compute(yawError, 0.0f, dt);
    }
}