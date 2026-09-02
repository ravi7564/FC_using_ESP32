#include "savemag.h"
#include <Preferences.h>

static const char* NVS_NAMESPACE = "fc_mag";
static const char* KEY_OFFSET_X  = "ox";
static const char* KEY_OFFSET_Y  = "oy";
static const char* KEY_OFFSET_Z  = "oz";
static const char* KEY_VALID     = "valid";

bool saveMagToNVS(int16_t xOff, int16_t yOff, int16_t zOff) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[SAVEMAG] ERROR: Cannot open NVS for write!");
        return false;
    }
    prefs.putShort(KEY_OFFSET_X, xOff);
    prefs.putShort(KEY_OFFSET_Y, yOff);
    prefs.putShort(KEY_OFFSET_Z, zOff);
    prefs.putBool(KEY_VALID, true);
    prefs.end();

    Serial.printf("[SAVEMAG] Offsets saved to NVS: X=%d Y=%d Z=%d\n", xOff, yOff, zOff);
    return true;
}

bool loadMagFromNVS(int16_t &xOff, int16_t &yOff, int16_t &zOff) {
    Preferences prefs;
    // Open in read/write mode so namespace is auto-created if fresh without throwing NOT_FOUND
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        return false;
    }
    bool valid = prefs.getBool(KEY_VALID, false);
    if (!valid) {
        prefs.end();
        return false;
    }
    xOff = prefs.getShort(KEY_OFFSET_X, 0);
    yOff = prefs.getShort(KEY_OFFSET_Y, 0);
    zOff = prefs.getShort(KEY_OFFSET_Z, 0);
    prefs.end();
    return true;
}

bool calibrateAndSaveMag(Magnetometer &mag, uint32_t durationMs) {
    Serial.println("\n========================================================");
    Serial.println("[SAVEMAG] Magnetometer Calibration Triggered via WiFi!");
    Serial.printf("[SAVEMAG] Duration: %lu seconds. Rotate plane in Figure-8 now!\n", durationMs / 1000);
    Serial.println("========================================================");

    // Temporarily clear offsets to capture true raw min/max
    mag.setCalibrationOffsets(0, 0, 0);

    int16_t minX = 32767,  minY = 32767,  minZ = 32767;
    int16_t maxX = -32768, maxY = -32768, maxZ = -32768;
    int count = 0;

    uint32_t start = millis();
    uint32_t lastPrint = 0;

    while (millis() - start < durationMs) {
        if (mag.update()) {
            MagData d = mag.getData();
            if (d.valid) {
                if (d.x < minX) minX = d.x;
                if (d.y < minY) minY = d.y;
                if (d.z < minZ) minZ = d.z;
                if (d.x > maxX) maxX = d.x;
                if (d.y > maxY) maxY = d.y;
                if (d.z > maxZ) maxZ = d.z;
                count++;
            }
        }
        delay(5);

        // Progress print every 2 seconds
        if (millis() - lastPrint >= 2000) {
            lastPrint = millis();
            uint32_t remaining = (durationMs - (millis() - start)) / 1000;
            Serial.printf("[SAVEMAG] Samples: %d | X:[%d,%d] Y:[%d,%d] Z:[%d,%d] | Time left: %lus\n",
                          count, minX, maxX, minY, maxY, minZ, maxZ, remaining);
        }
    }

    if (count < 20) {
        Serial.printf("[SAVEMAG] FAILED: Only %d samples collected. Check sensor connection!\n", count);
        // Restore previous offset if available
        int16_t prevX = 0, prevY = 0, prevZ = 0;
        if (loadMagFromNVS(prevX, prevY, prevZ)) {
            mag.setCalibrationOffsets(prevX, prevY, prevZ);
        }
        return false;
    }

    // Hard-iron center calculation
    int16_t xOff = (int16_t)((maxX + minX) / 2);
    int16_t yOff = (int16_t)((maxY + minY) / 2);
    int16_t zOff = (int16_t)((maxZ + minZ) / 2);

    Serial.println("\n[SAVEMAG] Calibration Successful!");
    Serial.printf("[SAVEMAG] Calculated Offsets: X=%d Y=%d Z=%d (%d samples)\n", xOff, yOff, zOff, count);

    // Apply immediately to live magnetometer
    mag.setCalibrationOffsets(xOff, yOff, zOff);

    // Save to NVS so next boot uses these updated offsets
    return saveMagToNVS(xOff, yOff, zOff);
}
