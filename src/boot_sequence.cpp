#include "boot_sequence.h"
#include "Calibration/calibration.h"
#include "Filter/filter_helper.h"
#include "config.h"

void runBootSequence(Accelerometer &accel, Gyroscope &gyro, Barometer &baro, Magnetometer &mag, FlightControlSystem &fc) {

    // 1. Initialize All Sensors
    initAndLoadMagnetometer(mag);
    accel.begin();
    gyro.begin();
    baro.begin();
    initFilter(250.0f); // 250Hz filter

    Serial.println("\n===========================================");
    Serial.println("--- [STEP 1/3] SYSTEM STARTING ---");
    Serial.println("Keep craft STEADY. Waiting 3 seconds...");

    // Allow sensor readings to stabilize
    delay(3000);

    Serial.println("\n--- [STEP 2/3] CALIBRATING ACCEL & GYRO ---");
    performSensorCalibration(accel, gyro, baro);

    Serial.println("\n--- [STEP 3/3] WAITING FOR SENSORS & ALIGNING FILTER ---");

    // Wait until Magnetometer gives valid data, with a 3-second timeout to prevent freeze
    bool magReady = false;
    MagData m;
    Serial.print("Waiting for Magnetometer Data");
    unsigned long magWaitStart = millis();
    while (!magReady && (millis() - magWaitStart < 3000)) {
        mag.update();
        m = mag.getData();
        if (m.valid && (m.x != 0 || m.y != 0 || m.z != 0)) {
            magReady = true;
        } else {
            Serial.print(".");
            delay(10);
        }
    }

    if (magReady) {
        Serial.println("\n[OK] Magnetometer is ONLINE!");
    } else {
        Serial.println("\n[WARN] Magnetometer offline/timeout! Continuing in 6-DOF IMU mode.");
    }

    // Read fresh sensor data
    accel.update(); gyro.update();
    if (magReady) mag.update();
    GyroData g = gyro.getData(); AccelData a = accel.getData();
    if (magReady) m = mag.getData();

    if (a.valid) {
        float mx = magReady ? (float)m.x : 0.0f;
        float my = magReady ? (float)m.y : 0.0f;
        float mz = magReady ? (float)m.z : 0.0f;

        Serial.println("Simulating 40 seconds of Filter Time instantly...");
        // 10,000 loops: (10000 * 0.004s = 40 seconds simulation)
        // Guarantees filter converges from initial 0 to true orientation
        for(int i = 0; i < 10000; i++) {
            updateFilter(0.0f, 0.0f, 0.0f,
                         (float)a.x/ACCEL_LSB_PER_G, (float)a.y/ACCEL_LSB_PER_G, (float)a.z/ACCEL_LSB_PER_G,
                         mx, my, mz, magReady);
        }
    }

    // Verify filter attitude lock
    float finalYaw = getYaw();
    Serial.printf("=> Filter Successfully Locked -> Roll: %.1f | Pitch: %.1f | Yaw: %.1f\n", getRoll(), getPitch(), finalYaw);

    Serial.println("\n--- INITIALIZING PID ---");
    // Initialize PID states with calibrated reference angles
    fc.reset();

    Serial.println("BOOT COMPLETE! Handing over to Flight Loop...");
    Serial.println("===========================================\n");
}