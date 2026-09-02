#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Sensors & Core Flight
#include "Sensors/Magnetometer/magnetometer.h"
#include "Sensors/Accel/accel.h"
#include "Sensors/Gyro/gyro.h"
#include "Sensors/Barometer/barometer.h"
#include "Calibration/calibration.h"
#include "Filter/filter_helper.h"
#include "pid.h"

// Boot & Data
#include "boot_sequence.h"
#include "crsf.h"
#include "datafeeder.h"

// Modules (Actuators, Mixer, Mode, Telemetry, Protocol, WiFi)
#include "mode.h"
#include "mixer.h"
#include "actuators.h"
#include "crsf_telemetry.h"
#include "protocol.h"
#include "wifi.h"

// ==========================================
// GLOBAL OBJECTS
// ==========================================
Magnetometer mag;
Accelerometer accel;
Gyroscope gyro;
Barometer baro;
FlightControlSystem fc;

crsfData_t crsfData;
DataFeeder rcFeeder;

ModeManager modeManager;
Mixer mixer;
Actuators actuators;

// CRSF Telemetry / Receiver Serial Port
HardwareSerial crsfSerial(1);

// Bench Testing: Set to false if testing without an active RC transmitter
const bool REQUIRE_RC = true;

// Angle & Rate limits from config
const float MAX_ROLL_ANGLE = MAX_ROLL_PITCH_DEG;   // 45.0f
const float MAX_PITCH_ANGLE = MAX_ROLL_PITCH_DEG;  // 45.0f
const float MAX_YAW_RATE = MAX_YAW_RATE_DPS;       // 360.0f

// Inter-Core Shared State (Safe variables shared between Core 1 and Core 0)
volatile float sharedAltitude = 0.0f;
volatile float sharedRoll = 0.0f;
volatile float sharedPitch = 0.0f;
volatile float sharedYaw = 0.0f;
volatile int16_t sharedThrottle = PWM_MIN;
volatile int16_t sharedAileron = PWM_CENTER;
volatile int16_t sharedElevator = PWM_CENTER;
volatile int16_t sharedRudder = PWM_CENTER;
volatile bool sharedArmed = false;
volatile bool sharedFailsafe = true;
volatile FlightMode sharedMode = MODE_STABILIZATION;

unsigned long lastFilterMicros = 0;

extern volatile bool request_calib_mag;

// ==========================================
// HELPER FUNCTIONS
// ==========================================

// Read real battery voltage from ADC pin with voltage divider ratio
float readBatteryVoltage() {
    uint32_t raw = analogRead(VBAT_PIN);
    float pinVoltage = ((float)raw / 4095.0f) * 3.3f;
    return pinVoltage * (float)BATTERY_DIVIDER_RATIO;
}

// PWM (1000-2000µs) to Angle/Rate Converter with center deadband
float mapPwmToAngle(uint16_t pwm, float maxAngle) {
    if (pwm > 1485 && pwm < 1515) return 0.0f;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    return ((float)((int)pwm - PWM_CENTER) / (float)(PWM_CENTER - PWM_MIN)) * maxAngle;
}

// Status LED Manager (Solid ON = Armed, Slow Pulse = Disarmed, Fast Pulse = Failsafe/Calib)
void updateStatusLED(bool armed, bool failsafe, bool calibActive) {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;
    unsigned long now = millis();

    if (calibActive) {
        // Fast Strobe during Magnetometer Calibration (50ms)
        if (now - lastBlinkTime >= 50) {
            lastBlinkTime = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
    } else if (failsafe) {
        // Fast Blink on Failsafe / Signal Lost (100ms)
        if (now - lastBlinkTime >= 100) {
            lastBlinkTime = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
    } else if (armed) {
        // Solid ON when Armed
        digitalWrite(LED_PIN, HIGH);
    } else {
        // Slow Pulse when Disarmed / Standby (500ms)
        if (now - lastBlinkTime >= 500) {
            lastBlinkTime = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
    }
}

// ==========================================
// CORE 0 TASK: Telemetry, WiFi & Feedback (Background)
// ==========================================
void core0Task(void *pvParameters) {
    unsigned long lastTelemetryTime = 0;
    unsigned long lastSerialTelemetryTime = 0;

    for (;;) {
        // 1. Handle Web Dashboard / WiFi
        handleWiFiClient();

        // 2. CRSF Telemetry to Transmitter (10Hz / 100ms)
        if (millis() - lastTelemetryTime >= 100) {
            lastTelemetryTime = millis();

            // Real battery voltage & live barometer altitude
            float batVolts = readBatteryVoltage();
            float altMeters = sharedAltitude;
            crsfSendSensorTelemetry(batVolts, altMeters);

            // Flight Mode Telemetry
            FlightMode mode = sharedMode;
            if (mode == MODE_MANUAL) crsfSendFlightMode("MANUAL");
            else if (mode == MODE_STABILIZATION) crsfSendFlightMode("STABILIZE");
            else crsfSendFlightMode("HORIZON");
        }

        // 3. USB Serial Telemetry / Debug Monitor (10Hz / 100ms)
        if (millis() - lastSerialTelemetryTime >= 100) {
            lastSerialTelemetryTime = millis();

            #if ACTIVE_TELEMETRY_MODE == TELEMETRY_MODE_BINARY
                uint8_t modeId = (sharedMode == MODE_MANUAL) ? 1 : ((sharedMode == MODE_HORIZON_LOCK) ? 2 : 0);
                uint8_t armState = sharedFailsafe ? 2 : (sharedArmed ? 1 : 0);
                sendBinaryTelemetry(
                    modeId,
                    armState,
                    sharedThrottle,
                    sharedAileron,
                    sharedElevator,
                    sharedRudder,
                    sharedAltitude,
                    sharedRoll,
                    sharedPitch,
                    sharedYaw
                );
            #else
                // Mode from ModeManager, Arm & Failsafe from DataFeeder, PWM from Mixer output, Angles from Boot-calibrated sensors
                const char* modeStr = (sharedMode == MODE_MANUAL) ? "MANUAL" : ((sharedMode == MODE_STABILIZATION) ? "STABILIZE" : "HORIZON");
                const char* armStr = sharedArmed ? "ARMED" : "DISARMED";
                const char* linkStr = sharedFailsafe ? "FAILSAFE" : "RC_OK";

                Serial.printf("Mode: %s %s %s | THR:%d AIL:%d ELE:%d RUD:%d | Alt: %.2fm | R:%.2f P:%.2f Y:%.2f\n",
                              modeStr, armStr, linkStr,
                              sharedThrottle, sharedAileron, sharedElevator, sharedRudder,
                              sharedAltitude, sharedRoll, sharedPitch, sharedYaw);
            #endif
        }

        // 4. Update Status LED
        updateStatusLED(sharedArmed, sharedFailsafe, request_calib_mag);

        // Reset watchdog & yield to FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// CORE 1: SETUP
// ==========================================
void setup() {
    Serial.begin(DEBUG_SERIAL_SPEED);

    // 1. Initialize Status LED & Actuators Early
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // [EARLY ACTUATOR INIT FOR ESC SAFETY]
    // Start actuators IMMEDIATELY with safe 1000us for motor and 1500us for servos
    // so ESC detects a valid low signal and doesn't beep or enter programming mode during sensor calibration
    actuators.begin(4, PWM_PINS, PWM_FREQ, PWM_RES);
    mixer.begin();

    // 2. Initialize I2C Bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);

    // 3. [PRIMARY BOOT SEQUENCE]
    // Initialize and calibrate sensors, align Madgwick AHRS, and reset PID states
    runBootSequence(accel, gyro, baro, mag, fc);

    // 4. [CRSF RECEIVER & WIRELESS SETUP]
    crsfSerial.begin(420000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    crsfInit();
    crsfTelemetryInit();
    initWiFi();

    // 5. [START CORE 0 BACKGROUND TASK]
    // Launch background Telemetry, WiFi AP, and Status LED task on Core 0
    xTaskCreatePinnedToCore(core0Task, "TelemetryTask", 4096, NULL, 1, NULL, 0);

    lastFilterMicros = micros();
}

// ==========================================
// CORE 1: MAIN FLIGHT LOOP (250Hz Real-Time)
// ==========================================
void loop() {
    // 1. Read CRSF Fast
    while (crsfSerial.available()) {
        uint8_t c = crsfSerial.read();
        if (crsfProcessByte(c, &crsfData)) {
            if (crsfData.valid) rcFeeder.feedCRSFData(true, crsfData.channels);
        }
    }

    // 2. Web Calibration Request (from WiFi dashboard)
    if (request_calib_mag) {
        request_calib_mag = false;
        // Motor zero for safety during calibration
        actuators.write(OUT_MOTOR, PWM_MIN);
        calibrateMagnetometer(mag, 6000);
        fc.reset();
        lastFilterMicros = micros();
    }

    // 3. 250Hz Flight Loop (4ms / 4000us interval with precise micros timing)
    uint32_t nowMicros = micros();
    if (nowMicros - lastFilterMicros >= 4000) {
        float dt = (float)(nowMicros - lastFilterMicros) * 1e-6f;
        lastFilterMicros = nowMicros;
        if (dt > 0.05f) dt = 0.004f; // Safety clamp against delay spikes

        mag.update();
        accel.update();
        gyro.update();
        baro.update();

        GyroData g = gyro.getData();
        AccelData a = accel.getData();
        MagData m = mag.getData();
        BaroData b = baro.getData();

        if (b.valid) {
            sharedAltitude = b.altitude_m;
        }

        if (g.valid && a.valid) {
            float mx = m.valid ? (float)m.x : 0.0f;
            float my = m.valid ? (float)m.y : 0.0f;
            float mz = m.valid ? (float)m.z : 0.0f;

            // A) Update Attitude Estimation Filter
            updateFilter(
                (float)g.gx / GYRO_LSB_PER_DPS,
                (float)g.gy / GYRO_LSB_PER_DPS,
                (float)g.gz / GYRO_LSB_PER_DPS,
                (float)a.x / ACCEL_LSB_PER_G,
                (float)a.y / ACCEL_LSB_PER_G,
                (float)a.z / ACCEL_LSB_PER_G,
                mx, my, mz, m.valid
            );

            float roll = getRoll();
            float pitch = getPitch();
            float yaw = getYaw();

            sharedRoll = roll;
            sharedPitch = pitch;
            sharedYaw = yaw;

            // B) RC Validity, Arming & Flight Mode Selection
            bool rcValid = rcFeeder.isValid();
            if (!REQUIRE_RC) rcValid = true; // Bench testing bypass

            bool isArmed = false;
            FlightMode currentMode = MODE_STABILIZATION;
            float roll_corr = 0.0f, pitch_corr = 0.0f, yaw_corr = 0.0f;
            float throttle_pwm = PWM_MIN;

            static bool armedState = false;

            if (rcValid) {
                // CH5 Arm Switch & Throttle Safety Check
                uint16_t armPwm = rcFeeder.getChannelPWM(RC_CH5_ARM);
                uint16_t rawThrottle = rcFeeder.getChannelPWM(RC_CH3_MOTOR);
                bool armSwitchOn = (armPwm > ARM_SWITCH_THRESHOLD);

                if (armSwitchOn) {
                    if (!armedState) {
                        // CRITICAL SAFETY: Only arm if throttle stick is at bottom (< 1050us)
                        if (rawThrottle <= 1050) {
                            armedState = true;
                        }
                    }
                } else {
                    // Arm switch OFF: Disarm immediately
                    armedState = false;
                }

                isArmed = armedState;

                // CH6 Flight Mode Selection
                uint16_t modePwm = rcFeeder.getChannelPWM(RC_CH6_FLIGHTMODE);
                modeManager.update(modePwm);
                currentMode = modeManager.getMode();

                // Throttle: Only active when armed
                if (isArmed) {
                    throttle_pwm = rawThrottle;
                } else {
                    throttle_pwm = PWM_MIN;
                }

                if (currentMode == MODE_MANUAL) {
                    // MANUAL MODE: Direct pilot sticks pass-through (PID bypassed)
                    roll_corr  = (float)rcFeeder.getChannelPWM(RC_CH1_AILERON) - (float)PWM_CENTER;
                    pitch_corr = (float)rcFeeder.getChannelPWM(RC_CH2_ELEVATOR) - (float)PWM_CENTER;
                    yaw_corr   = (float)rcFeeder.getChannelPWM(RC_CH4_RUDDER) - (float)PWM_CENTER;
                } else {
                    // STABILIZE / HORIZON MODE: PID angle stabilization
                    float rcRoll  = mapPwmToAngle(rcFeeder.getChannelPWM(RC_CH1_AILERON), MAX_ROLL_ANGLE);
                    float rcPitch = mapPwmToAngle(rcFeeder.getChannelPWM(RC_CH2_ELEVATOR), MAX_PITCH_ANGLE);
                    float rcYaw   = (float)rcFeeder.getChannelPWM(RC_CH4_RUDDER) - (float)PWM_CENTER;

                    float pidRollOut = 0.0f, pidPitchOut = 0.0f, pidYawOut = 0.0f;
                    fc.computeControl(rcRoll, rcPitch, rcYaw, roll, pitch, yaw, dt,
                                      pidRollOut, pidPitchOut, pidYawOut);

                    roll_corr = pidRollOut;
                    pitch_corr = pidPitchOut;
                    yaw_corr = pidYawOut;
                }
            } else {
                // FAILSAFE / BENCH TEST (No RC Signal):
                // Motor is strictly CUT OFF (1000µs) for safety!
                isArmed = false;
                armedState = false;
                throttle_pwm = PWM_MIN;
                currentMode = MODE_STABILIZATION;

                // Active Auto-Level Stabilization (Target: 0° Roll, 0° Pitch, 0° Yaw change)
                float pidRollOut = 0.0f, pidPitchOut = 0.0f, pidYawOut = 0.0f;
                fc.computeControl(0.0f, 0.0f, 0.0f, roll, pitch, yaw, dt,
                                  pidRollOut, pidPitchOut, pidYawOut);

                roll_corr = pidRollOut;
                pitch_corr = pidPitchOut;
                yaw_corr = pidYawOut;
            }

            // Update shared variables for Core 0 Telemetry & LED
            sharedArmed = isArmed;
            sharedFailsafe = !rcValid;
            sharedMode = currentMode;

            // C) MIXER: Convert PID corrections to physical motor and servo PWM commands
            MixerOutputs mixOut = mixer.mix(throttle_pwm, roll_corr, pitch_corr, yaw_corr);

            // Safety override: Disarmed or failsafe ensures motor is OFF
            if (!isArmed) {
                mixOut.throttle_us = PWM_MIN;
            }

            // D) ACTUATORS: Output to physical PWM GPIO pins
            int finalOutputs[4];
            finalOutputs[OUT_AILERON]  = mixOut.aileron_us;
            finalOutputs[OUT_ELEVATOR] = mixOut.elevator_us;
            finalOutputs[OUT_MOTOR]    = mixOut.throttle_us;
            finalOutputs[OUT_RUDDER]   = mixOut.rudder_us;

            actuators.updateFromMixer(finalOutputs, 4);

            // Update shared output telemetry
            sharedThrottle = mixOut.throttle_us;
            sharedAileron  = mixOut.aileron_us;
            sharedElevator = mixOut.elevator_us;
            sharedRudder   = mixOut.rudder_us;
        }
    }
}