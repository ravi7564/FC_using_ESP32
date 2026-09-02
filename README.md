# ESP32 Flight Controller Architecture

> A high-performance, dual-core ESP32 flight controller built for fixed-wing and delta-wing aircraft, featuring ExpressLRS (CRSF) telemetry, 9-DOF Madgwick AHRS sensor fusion, and real-time FreeRTOS task separation.

---

## System Architecture Flow

```text
[ HARDWARE INPUTS ]
┌───────────────────────────────┐                             ┌──────────────────────────────────────────────┐
│     CRSF / ELRS RECEIVER      │                             │           I2C SENSOR BUS (400kHz)            │
│   (UART2: RX=16 / TX=17)      │                             │            (SDA=32 / SCL=33)                 │
└───────────────┬───────────────┘                             └──────────────────────┬───────────────────────┘
                │ Serial Stream (420k baud)                                          │ MPU6500, MMC5983, BMP280
                ▼                                                                    ▼
┌───────────────────────────────┐                             ┌──────────────────────────────────────────────┐
│          crsf.cpp             │                             │               SENSOR DRIVERS                 │
│  - CRC Check (0xD5 Poly)      │                             │  - Gyroscope (±2000°/s, DLPF 42Hz)           │
│  - Extract 16 Channels        │                             │  - Accelerometer (±8G, 4096 LSB/G)           │
└───────────────┬───────────────┘                             │  - Magnetometer (MMC 100Hz, Hard-iron Offsets)│
                │ 11-bit Raw (172..1811)                      │  - Barometer (BMP280 Altitude / Pressure)    │
                ▼                                             └──────────────────────┬───────────────────────┘
┌───────────────────────────────┐                                                    │ Calibrated Gyro/Accel/Mag Data
│          DataFeeder           │                                                    ▼
│  - CRSF -> PWM (1000..2000µs) │                             ┌──────────────────────────────────────────────┐
│  - Timeout Check (< 300ms)    │                             │            MADGWICK AHRS FILTER              │
└───────────────┬───────────────┘                             │     (250Hz Real-Time Attitude Fusion)        │
                │                                             └──────────────────────┬───────────────────────┘
     isValid? (RC Link State)                                                        │
                │                                                        Estimated Attitude Angles
     ┌──────────┴──────────┐                                             (Roll, Pitch, Heading Yaw)
     ▼                     ▼                                                         │
[ FAILSAFE: TRUE ]    [ FAILSAFE: FALSE ]                                              │
(Signal Lost / Off)   (RC Signal Healthy)                                              │
     │                     │                                                         │
     │              Read Channels:                                                   │
     │              - CH1: Aileron    (Roll)                                         │
     │              - CH2: Elevator   (Pitch)                                        │
     │              - CH3: Throttle   (Motor)                                        │
     │              - CH4: Rudder     (Yaw)                                          │
     │              - CH5: Arm Switch (1700µs)                                       │
     │              - CH6: Mode Switch                                               │
     │                     │                                                         │
     │              ┌──────┴─────────────────────────────────┐                       │
     │              │  ARMING SAFETY CHECK                   │                       │
     │              │  Switch > 1700µs & Throttle <= 1050µs? │                       │
     │              └──────┬─────────────────────────────────┘                       │
     │                     │                                                         │
     │             ┌───────┴───────────────┐                                         │
     │             ▼                       ▼                                         │
     │        [ DISARMED ]              [ ARMED ]                                    │
     │      Throttle = 1000µs       Throttle = Active                                │
     │             │                       │                                         │
     │             └───────┬───────────────┘                                         │
     │                     │                                                         │
     │              Flight Mode Check (CH6):                                         │
     │              ┌──────┴───────────────────────┐                                 │
     │              ▼                              ▼                                 │
     │      [ MODE_MANUAL ]             [ MODE_STABILIZATION ]                       │
     │      (1300µs - 1700µs)           (< 1300µs / > 1700µs)                        │
     │              │                              │                                 │
     │       Sticks Direct                         ▼                                 │
     │       (PID Bypassed)              Target Stick Angles:                        │
     │              │                    - Roll  Target: -45° to +45°                │
     │              │                    - Pitch Target: -45° to +45°                │
     │              │                    - Yaw Rate / Heading Lock                   │
     │              │                              │                                 │
     │              │                              ▼                                 │
     │              │               ┌──────────────────────────────┐                 │
     │              │               │     FlightControlSystem      │                 │
     │              │               │        (lib/PID/pid)         │◄────────────────┘
     │              │               │  - Roll  PID (P:2.0, D-LPF)  │  Live Attitude
     │              │               │  - Pitch PID (P:2.2, D-LPF)  │  (Roll, Pitch, Yaw)
     │              │               │  - Yaw Coordinated Lock      │
     │              │               └──────────────┬───────────────┘
     │              │                              │
     │              │                     PID Corrections:
     │              │                     (roll_corr, pitch_corr, yaw_corr)
     │              │                              │
     └──────────────┼──────────────────────────────┘
                    │
                    ▼
    ┌────────────────────────────────┐
    │             MIXER              │
    │   (ACTIVE_WING_TYPE Config)    │
    └───────────────┬────────────────┘
                    │
   ┌────────────────┴────────────────┐
   ▼                                 ▼
[ STANDARD FIXED-WING ]          [ DELTA-WING / ELEVON ]
AIL = 1500 + Roll                Left  = 1500 + Pitch + Roll
ELE = 1500 - Pitch               Right = 1500 + Pitch - Roll
RUD = 1500 + Yaw                 RUD   = 1500 + Yaw
THR = Throttle (Min 1000)        THR   = Throttle (Min 1000)
   │                                 │
   └────────────────┬────────────────┘
                    │
                    ▼ MixerOutputs (µs: 1000..2000)
    ┌────────────────────────────────┐
    │           ACTUATORS            │
    │    (lib/Actuators/actuators)   │
    │   - ESP32 LEDC PWM Peripheral  │
    │   - 50Hz Frequency / 14-Bit    │
    └───────────────┬────────────────┘
                    │
                    ▼ Duty Cycle Signals
    ┌────────────────────────────────┐
    │       PHYSICAL GPIO PINS       │
    ├────────────────┬───────────────┤
    │  GPIO 27       │  OUT_MOTOR    │ ──► [ ESC / BRUSHLESS MOTOR ]
    │  GPIO 13       │  OUT_AILERON  │ ──► [ AILERON SERVO (Roll)  ]
    │  GPIO 14       │  OUT_ELEVATOR │ ──► [ ELEVATOR SERVO (Pitch)]
    │  GPIO 18       │  OUT_RUDDER   │ ──► [ RUDDER SERVO (Yaw)    ]
    └────────────────┴───────────────┘

========================================================================================================================
                          CORE 0: BACKGROUND TASK (10Hz / 100ms FreeRTOS Async)
========================================================================================================================

             Inter-Core Shared State (volatile float sharedRoll, sharedPitch, sharedAlt, etc.)
                                                       │
         ┌─────────────────────────────────────────────┼─────────────────────────────────────────────┐
         ▼                                             ▼                                             ▼
┌───────────────────────────────┐             ┌───────────────────────────────┐             ┌───────────────────────────────┐
│        CRSF TELEMETRY         │             │         USB TELEMETRY         │             │      WIFI ACCESS POINT        │
│   (TX Pin 17 -> Receiver)     │             │       (115200 Baud USB)       │             │       (SSID: FC-Config)       │
├───────────────────────────────┤             ├───────────────────────────────┤             ├───────────────────────────────┤
│ - Real Battery Voltage (ADC34)│             │ - Binary Frame Mode ($M...)   │             │ - Web Dashboard (192.168.4.1) │
│ - Baro Altitude (Meters)      │             │   (For 3D Attitude Visualizer)│             │ - Remote Magnetometer         │
│ - Flight Mode Name (STABILIZE)│             │ - Text Debug Strings          │             │   Live Calibration Trigger    │
└───────────────────────────────┘             └───────────────────────────────┘             └───────────────────────────────┘



Key Workflow Breakdown
1. Boot & Early Safety Sequence (setup())
Actuators First: Immediately upon power-up, GPIO 27 (Motor ESC) is driven to a safe 1000µs signal and Servos to a centered 1500µs signal. This prevents continuous ESC error beeping, accidental programming mode triggers, or servo glitches during boot.

Sensor Calibration: Calibrates the IMU (Gyroscope/Accelerometer) and Barometer with built-in 4-second timeout protection to ensure the flight controller never hangs if a sensor fails or disconnects.

Madgwick Filter Warmup: Fast-forwards 10,000 simulated iterations at boot so the Madgwick AHRS filter instantly converges and locks onto the true heading/attitude before handing over to the flight loop.

2. Core 1: Real-Time Flight Loop (250Hz / 4000µs)
Microsecond Timing: Uses jitter-free micros() timing to poll and update Gyroscope, Accelerometer, Magnetometer, and Barometer data every 4ms.

Attitude Estimation: Fuses calibrated 9-DOF/6-DOF sensor data through the Madgwick AHRS filter to compute real-time Roll, Pitch, and Yaw attitude angles.

DataFeeder & Failsafe: Continuously decodes CRSF receiver frames. If signal is lost or packet arrival exceeds 300ms, Failsafe immediately triggers: motor is cut off (1000µs) and wings-level auto-glide stabilization is maintained.

Zero-Throttle Arming Safety: A stateful safety interlock prevents the craft from arming unless the throttle stick is at minimum (<= 1050µs) when the Arm switch is activated.

PID Stabilization: Fixed-Wing tuned PID gains (P = 2.0 Roll, P = 2.2 Pitch) paired with a 20Hz derivative low-pass filter eliminate motor vibration noise and prevent servo chattering/buzzing. Includes bank-coordinated Heading Hold on Yaw to eliminate tail wagging in turns.

Mixer: Distributes throttle and PID corrections to actuators according to the selected airframe configuration (Standard Fixed-Wing or Delta-Wing Elevons) and outputs clean 50Hz, 14-bit LEDC PWM signals.

3. Core 0: Background Telemetry & Connectivity (10Hz / 100ms)
CRSF Telemetry: Asynchronously transmits real-time battery voltage (ADC GPIO 34), barometric altitude (BMP280), and active flight mode back to the pilot's transmitter without interrupting Core 1's real-time flight loop.

USB Telemetry: Streams high-speed binary telemetry frames ($M...) over Serial for real-time 3D Ground Station attitude visualization and debug logging.

WiFi Access Point: Hosts an onboard Web Dashboard (SSID: FC-Config @ 192.168.4.1), allowing pilots to wirelessly trigger and save hard-iron magnetometer calibration from any smartphone or laptop on the ground.