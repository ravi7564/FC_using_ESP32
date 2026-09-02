#include "calibration.h"
#include <Preferences.h>
#include "config.h"

void initAndLoadMagnetometer(Magnetometer &mag) {
    if (mag.begin()) {
        Preferences preferences;
        preferences.begin("mag_cal", true); // Read-only mode
        int16_t xOff = preferences.getShort("x_off", 0);
        int16_t yOff = preferences.getShort("y_off", 0);
        int16_t zOff = preferences.getShort("z_off", 0);
        preferences.end();

        mag.setCalibrationOffsets(xOff, yOff, zOff);
        Serial.println("[BOOT] Magnetometer initialized.");
        Serial.print("[BOOT] Loaded Mag Offsets -> X: "); Serial.print(xOff);
        Serial.print(" Y: "); Serial.print(yOff);
        Serial.print(" Z: "); Serial.println(zOff);
    } else {
        Serial.println("[BOOT] ERROR: Magnetometer initialization failed!");
    }
}

// Magnetometer Live Calibration (triggered via WiFi Web Dashboard)
bool calibrateMagnetometer(Magnetometer &mag, uint32_t durationMs) {
    Serial.println("\n========================================================");
    Serial.println("[CALIB] Magnetometer Calibration Started!");
    Serial.printf("[CALIB] Duration: %lu seconds. Rotate plane in Figure-8 now!\n", durationMs / 1000);
    Serial.println("========================================================");

    // Clear previous offsets to obtain raw sensor readings
    mag.setCalibrationOffsets(0, 0, 0);

    int16_t xMin = 32767, xMax = -32768;
    int16_t yMin = 32767, yMax = -32768;
    int16_t zMin = 32767, zMax = -32768;
    int count = 0;

    unsigned long startTime = millis();
    unsigned long lastPrint = 0;

    while (millis() - startTime < durationMs) {
        if (mag.update()) {
            MagData m = mag.getData();
            if (m.valid) {
                if (m.x < xMin) xMin = m.x;
                if (m.x > xMax) xMax = m.x;
                if (m.y < yMin) yMin = m.y;
                if (m.y > yMax) yMax = m.y;
                if (m.z < zMin) zMin = m.z;
                if (m.z > zMax) zMax = m.z;
                count++;
            }
        }
        delay(5);

        // Print progress update every 2 seconds
        if (millis() - lastPrint >= 2000) {
            lastPrint = millis();
            uint32_t remaining = (durationMs - (millis() - startTime)) / 1000;
            Serial.printf("[CALIB] Samples: %d | X:[%d,%d] Y:[%d,%d] Z:[%d,%d] | Time left: %lus\n",
                          count, xMin, xMax, yMin, yMax, zMin, zMax, remaining);
        }
    }

    if (count < 20) {
        Serial.printf("\n[CALIB] FAILED: Only %d samples collected. Restoring old offsets.\n", count);
        initAndLoadMagnetometer(mag); // Restore previously saved offsets
        return false;
    }

    // Hard-iron offsets calculation
    int16_t xOff = (xMin + xMax) / 2;
    int16_t yOff = (yMin + yMax) / 2;
    int16_t zOff = (zMin + zMax) / 2;

    // Save offsets to NVS
    Preferences preferences;
    preferences.begin("mag_cal", false); // Read-write mode
    preferences.putShort("x_off", xOff);
    preferences.putShort("y_off", yOff);
    preferences.putShort("z_off", zOff);
    preferences.end();

    mag.setCalibrationOffsets(xOff, yOff, zOff);

    Serial.println("\n[CALIB] Calibration Successful & Saved to NVS!");
    Serial.printf("[CALIB] New Offsets -> X:%d Y:%d Z:%d (%d samples)\n", xOff, yOff, zOff, count);
    return true;
}

void performSensorCalibration(Accelerometer &accel, Gyroscope &gyro, Barometer &baro) {
    Serial.println("Starting Accel, Gyro, and Baro calibration... Keep the craft STEADY!");

    const int numSamples = 500;
    int32_t sumGx = 0, sumGy = 0, sumGz = 0;
    int32_t sumAx = 0, sumAy = 0, sumAz = 0;
    double sumPressure = 0;
    int validSamplesIMU = 0;
    int validSamplesBaro = 0;

    unsigned long startTime = millis();
    while (validSamplesIMU < numSamples && (millis() - startTime < 4000)) {
        if (gyro.update() && accel.update()) {
            GyroData gData = gyro.getData();
            AccelData aData = accel.getData();
            if (gData.valid && aData.valid) {
                sumGx += gData.gx;
                sumGy += gData.gy;
                sumGz += gData.gz;

                sumAx += aData.x;
                sumAy += aData.y;
                sumAz += aData.z;
                validSamplesIMU++;
            }
        }

        if (baro.update()) {
            BaroData bData = baro.getData();
            if (bData.valid) {
                sumPressure += bData.pressure_pa;
                validSamplesBaro++;
            }
        }
        delay(3);
    }

    if (validSamplesIMU > 0) {
        int16_t gOffX = sumGx / validSamplesIMU;
        int16_t gOffY = sumGy / validSamplesIMU;
        int16_t gOffZ = sumGz / validSamplesIMU;

        int16_t aOffX = sumAx / validSamplesIMU;
        int16_t aOffY = sumAy / validSamplesIMU;
        int16_t aOffZ = (sumAz / validSamplesIMU) - ACCEL_LSB_PER_G;

        gyro.setCalibrationOffsets(gOffX, gOffY, gOffZ);
        accel.setCalibrationOffsets(aOffX, aOffY, aOffZ);

        Serial.println("IMU Calibration completed successfully!");
        Serial.print("Gyro Offsets  -> X: "); Serial.print(gOffX); Serial.print(" Y: "); Serial.print(gOffY); Serial.print(" Z: "); Serial.println(gOffZ);
        Serial.print("Accel Offsets -> X: "); Serial.print(aOffX); Serial.print(" Y: "); Serial.print(aOffY); Serial.print(" Z: "); Serial.println(aOffZ);
    } else {
        Serial.println("[ERROR] IMU Calibration failed! Sensor not responding.");
    }

    if (validSamplesBaro > 0) {
        float avgPressure = sumPressure / validSamplesBaro;
        float pressureOffset = 101325.0f - avgPressure;
        baro.setCalibrationOffset(pressureOffset);
        Serial.print("Baro Pressure Offset: "); Serial.println(pressureOffset);
    } else {
        Serial.println("[WARN] Barometer offline or no samples during calibration.");
    }
}