//include/config.h

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============= HARDWARE PIN CONFIGURATION =============
// Battery & Power
#define VBAT_PIN                34

// RC Receiver (CRSF)
#define CRSF_RX_PIN             16
#define CRSF_TX_PIN             17

// I2C Sensors (MPU6500, BMP280, HMC5883L)
#define I2C_SDA_PIN             32
#define I2C_SCL_PIN             33
#define I2C_FREQ                400000  // 400kHz standard mode

// Output & Feedback
#define LED_PIN                 2

// ============= RC CHANNEL MAPPING (0-based, for sharedChannels[]) =============
#define RC_CH1_AILERON          0
#define RC_CH2_ELEVATOR         1
#define RC_CH3_MOTOR            2
#define RC_CH4_RUDDER           3
#define RC_CH5_ARM              4    // Logic only, no physical pin
#define RC_CH6_FLIGHTMODE       5    // Logic only, no physical pin
#define RC_CH7_AUX1             6
#define RC_CH8_AUX2             7
#define RC_CH9_AUX3             8
#define RC_CH10_AUX4            9

// ============= PWM OUTPUT MAPPING (0-based, for actuators.write()) =============
#define OUT_AILERON             0
#define OUT_ELEVATOR            1
#define OUT_MOTOR               2
#define OUT_RUDDER              3
#define OUT_AUX1                4
#define OUT_AUX2                5
#define OUT_AUX3                6
#define OUT_AUX4                7

// ============= OUTPUT: PWM Format (1000-2000) =============
#define PWM_MIN                 1000
#define PWM_CENTER              1500
#define PWM_MAX                 2000

#define TOTAL_CHANNELS          8
#define ARM_SWITCH_THRESHOLD    1700

// Physical GPIO pins for PWM output (same order as OUT_* defines)
static const int PWM_PINS[TOTAL_CHANNELS] = {
    13, 14, 27, 18, 19, 21, 25, 26
};

// ============= PWM CONFIGURATION =============
#define PWM_FREQ                50
#define PWM_RES                 14
#define BATTERY_DIVIDER_RATIO   11.0

// Inline function: Convert CRSF value (172-1811) to PWM microseconds (1000-2000)
inline int calculate_pwm_us(uint16_t crsf_val) {
    if (crsf_val <= 172) return PWM_MIN;
    if (crsf_val >= 1811) return PWM_MAX;
    return (int)(((uint32_t)(crsf_val - 172) * (PWM_MAX - PWM_MIN)) / (1811 - 172)) + PWM_MIN;
}

// ============= WING CONFIGURATION =============
#define WING_TYPE_STANDARD      1
#define WING_TYPE_DELTA         2

// Select your active wing configuration
#define ACTIVE_WING_TYPE        WING_TYPE_STANDARD


// ============= SENSOR SAMPLING RATES =============
#define IMU_SAMPLE_RATE_HZ      100      // MPU6050 update rate
#define IMU_INTERVAL_US         10000    // 1000000 / 100 = 10000 microseconds

#define BARO_SAMPLE_RATE_HZ     50       // BMP280 update rate
#define BARO_INTERVAL_US        20000    // 1000000 / 50 = 20000 microseconds

#define MAG_SAMPLE_RATE_HZ      100      // HMC5883L update rate
#define MAG_INTERVAL_US         10000    // 1000000 / 100 = 10000 microseconds

// ============= CALIBRATION & COMPENSATION =============
// Accelerometer gravity constant in LSB units (at ±8G, 1G = 4096 LSB)
#define GRAVITY_LSB             4096.0f

// Barometer sea level pressure (Pa)
#define SEA_LEVEL_PRESSURE_PA   101325.0f

// ============= FLIGHT CONTROLLER LOOP TIMING =============
#define LOOPTIME_US             10000    // 10ms main loop (100Hz)
#define MAX_ROLL_PITCH_DEG      45       // Max tilt angle
#define MAX_YAW_RATE_DPS        360      // Max rotation rate

// ============= DEBUG & MONITORING =============
#define DEBUG_SERIAL_SPEED      115200
#define ENABLE_SERIAL_DEBUG     1
#define ENABLE_I2C_DEBUG        0

// ============= USB SERIAL TELEMETRY MODE =============
#define TELEMETRY_MODE_TEXT      1   // Human-readable Serial Monitor prints
#define TELEMETRY_MODE_BINARY    2   // High-speed Binary Frame ($M + 26-byte payload) for 3D Visualizer

// Set your desired output mode here:
#define ACTIVE_TELEMETRY_MODE   TELEMETRY_MODE_TEXT

// ============= WIFI ACCESS POINT CONFIGURATION =============
#define WIFI_AP_SSID            "FC-Config"
#define WIFI_AP_PASSWORD        "12345678"
#define WIFI_AP_TIMEOUT_MS      60000    // 1 minute auto-shutdown if no client

#endif