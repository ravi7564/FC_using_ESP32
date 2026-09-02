#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"

// Only for magnetometer calibration request from web UI
extern volatile bool request_calib_mag;

// Functions
void initWiFi();
void handleWiFiClient();

#endif // WIFI_MANAGER_H