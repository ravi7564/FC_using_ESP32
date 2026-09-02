#include "wifi.h"

WebServer server(80);

// --- Timeout Variables ---
unsigned long wifiStartTime = 0;
const unsigned long wifiTimeout = WIFI_AP_TIMEOUT_MS;
bool wifiIsActive = false;
bool clientConnected = false;

// Magnetometer calibration request flag from Web UI
volatile bool request_calib_mag = false;

// Web UI Dashboard for Magnetometer Calibration
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>FC Configurator</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background-color: #121212; color: #fff; margin: 0; padding: 20px; }
        h1 { font-size: 24px; margin-bottom: 20px; }

        .card { background: #242424; padding: 20px; margin: 15px auto; border-radius: 12px; width: 90%; max-width: 400px; box-shadow: 0px 4px 10px rgba(0,0,0,0.5); text-align: left;}

        .btn { padding: 15px 20px; font-size: 16px; font-weight: bold; color: white; border: none; border-radius: 8px; cursor: pointer; transition: 0.3s; width: 100%; text-transform: uppercase;}
        .btn-mag { background-color: #ff8800; }
        .btn:hover { opacity: 0.8; }
        .btn:disabled { background-color: #555; cursor: not-allowed; color: #888;}

        .instruction { font-size: 14px; color: #bbb; margin-top: 15px; line-height: 1.4; }
        .status { font-weight: bold; margin-top: 15px; font-size: 15px; min-height: 20px; text-align: center; }
    </style>
</head>
<body>
    <h1>Plane Controller Setup</h1>

    <!-- Magnetometer Card -->
    <div class="card">
        <button id="btn-mag" class="btn btn-mag" onclick="calibrateMag()">Calibrate Magnetometer</button>
        <div class="instruction">
            <b>Instruction:</b> Press button and rotate plane in all axes (Figure-8 pattern) for 7 seconds.<br><br>
            <i>(Note: Accel & Gyro calibrate automatically at boot)</i>
        </div>
        <div id="status-mag" class="status"></div>
    </div>

    <script>
        // Background ping to keep WiFi alive while dashboard is open
        setInterval(() => fetch('/ping'), 10000);

        function calibrateMag() {
            let btn = document.getElementById('btn-mag');
            let status = document.getElementById('status-mag');

            btn.disabled = true; // Lock button
            status.style.color = "#f39c12"; // Yellow/Orange
            status.innerHTML = "⏳ Calibrating... ROTATE PLANE IN ALL AXES!";

            fetch('/calibrate_mag').then(response => {
                // Calibration routine takes 6 seconds. UI waits 7 seconds before completion.
                setTimeout(() => {
                    status.style.color = "#28a745"; // Green
                    status.innerHTML = "✅ Magnetometer Saved Successfully!";
                    btn.disabled = false; // Button unlock
                }, 7000);
            });
        }
    </script>
</body>
</html>
)rawliteral";


void initWiFi() {
    Serial.println("\n[WiFi] Starting Access Point (Hotspot) Mode...");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

    Serial.print("[WiFi] Network Name (SSID): ");
    Serial.println(WIFI_AP_SSID);
    Serial.print("[WiFi] Dashboard URL: http://");
    Serial.println(WiFi.softAPIP());

    wifiIsActive = true;
    wifiStartTime = millis();

    // --- Web Routes ---

    server.on("/", []() {
        clientConnected = true;
        wifiStartTime = millis();
        server.send(200, "text/html", htmlPage);
    });

    server.on("/ping", []() {
        clientConnected = true;
        wifiStartTime = millis();
        server.send(200, "text/plain", "pong");
    });

    server.on("/calibrate_mag", []() {
        wifiStartTime = millis();
        server.send(200, "text/plain", "OK");
        request_calib_mag = true; // Trigger in main.cpp
    });

    server.begin();
    Serial.println("[WiFi] Web server running. Connect your phone now!");
}


void handleWiFiClient() {
    if (!wifiIsActive) return;

    server.handleClient();

    // 1 Minute Inactivity Check
    if (millis() - wifiStartTime > wifiTimeout) {
        Serial.println("\n[WiFi] 1 Minute Inactivity Timeout Reached!");
        Serial.println("[WiFi] Turning OFF WiFi to save power and CPU...");

        server.stop();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);

        wifiIsActive = false;
    }
}