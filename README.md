# Flight_controller_using_esp32

========================================================================================================================
                                      ESP32 FLIGHT CONTROLLER ARCHITECTURE FLOW
========================================================================================================================

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

═════════════════════════════════════════════════════════════════════════════
                     [ CORE 0 ASYNC FEEDBACK TASK ]
   Shared Data ──► Status LED (GPIO 2) : Solid ON (STBL) / Blink (Failsafe)
               ──► CRSF Telemetry      : Altitude & Flight Mode
               ──► USB Serial Monitor  : 100ms Live Telemetry Prints

BENCH TESTING TRICK:
If this is toggled, it will operate directly in active stabilization mode without the remote.
Keep this set to 'True' for the flight.
const bool REQUIRE_RC = true; change true to faalse for  testing