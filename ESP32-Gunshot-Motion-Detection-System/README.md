# IoT Acoustic Event & Intrusion Detection System

##Overview
This project is a sophisticated embedded system prototype designed to detect environmental anomalies and security threats at the "Edge." Built on the ESP32 platform, it utilizes multi-modal sensing to detect specific audio signatures (like loud transient events), human motion, and physical device tampering.

Unlike standard motion detectors, this system employs **Digital Signal Processing (DSP)** to analyze audio frequencies in real-time, distinguishing between potential threats and ambient noise before transmitting data securely to a remote server.

> **Note:** This is a prototype version of a  project that i am working on .



## Key Features

### 1. Advanced Acoustic Analysis (Gunshot/Impact Detection)
* **Real-time Sampling:** Uses an I2S MEMS microphone sampling at 16kHz on a dedicated CPU core.
* **Signal Processing:** Implements FFT (Fast Fourier Transform) to analyze frequency distribution.
* **Classification Logic:** Uses a combination of High/Low frequency ratios, Zero Crossing Rate (ZCR), and event duration to validate triggers.
* **Noise Filtering:** Rejects events with excessive bass (like thunder) or incorrect duration.

### 2. Sensor Fusion Motion Detection
* **Dual-Technology:** Combines **PIR (Passive Infrared)** and **RCWL (Microwave Radar)** sensors.
* **Logic:** Requires simultaneous triggers from both sensors to confirm motion, significantly reducing false alarms caused by heat changes or wind.

### 3. Anti-Tamper Mechanism
* Utilizes an **MPU6050 Accelerometer/Gyroscope**.
* Calculates 3D tilt angles relative to a calibrated reference vector.
* Triggers an immediate alert if the device is knocked over or rotated more than 30 degrees.

### 4. Robust IoT Connectivity
* **Secure Communication:** Sends JSON payloads via HTTPS (SSL/TLS).
* **WiFi Provisioning:** Features a Captive Portal (WiFiManager) for easy network configuration without hardcoding credentials.
* **Health Monitoring:** Sends periodic "Heartbeat" signals containing RSSI signal strength, free heap memory, and system uptime.

## Hardware Stack
* **Microcontroller:** ESP32 Development Board (Dual Core, 240MHz)
* **Audio:** INMP441 Omnidirectional I2S Microphone
* **IMU:** MPU6050 (6-Axis Accelerometer/Gyro)
* **Motion:** HC-SR501 (PIR) + RCWL-0516 (Microwave Radar)
* **Interface:** Status LEDs and Calibration Buttons



## Software & Libraries
* **Framework:** Arduino for ESP32
* **OS:** FreeRTOS (Task scheduler for multi-threading)
* **DSP:** `arduinoFFT` for frequency analysis
* **Sensors:** `Adafruit_MPU6050`, `Adafruit_Sensor`
* **Network:** `WiFiClientSecure`, `HTTPClient`, `WiFiManager`

## How It Works (The Core Logic)

### The Audio Pipeline (Core 0)
1.  **Listen:** Continuously fills a DMA buffer with audio samples.
2.  **Trigger:** If amplitude exceeds a threshold, it records the "event."
3.  **Process:** If the event length matches specific criteria (8-35 chunks), it performs an FFT.
4.  **Analyze:**
    * *Ratio Check:* Ensures high-frequency energy is significantly higher than low-frequency energy.
    * *ZCR Check:* Ensures the signal crosses zero frequently (indicative of percussive sounds).
5.  **Alert:** If confirmed, a flag is passed to Core 1.

### The Sensor Loop (Core 1)
1.  **Monitor:** Polls Motion and IMU sensors.
2.  **Heartbeat:** Checks system health every 5 seconds.
3.  **Dispatch:** Formats JSON data and transmits to the configured Flask/Cloud server.

##  Pin Configuration

| Component | Pin (ESP32) |
| :--- | :--- |
| **I2S Mic WS** | GPIO 15 |
| **I2S Mic SD** | GPIO 33 |
| **I2S Mic SCK**| GPIO 14 |
| **PIR Sensor** | GPIO 27 |
| **Radar (RCWL)**| GPIO 17 |
| **SDA (MPU6050)**| GPIO 21 (Default) |
| **SCL (MPU6050)**| GPIO 22 (Default) |
| **Status LED** | GPIO 2 |

## Installation & Setup
1.  **Wiring:** Connect components according to the pinout above.
2.  **Flash:** Upload the code using Arduino IDE or PlatformIO.
3.  **Provision:**
    * On first boot, the device creates a WiFi Hotspot named `ESP32-Security`.
    * Connect to it and configure your home WiFi and Backend Server URL.
4.  **Calibrate:** Place the device in its final position and press the Calibration Button (GPIO 4) to set the reference angle.

## Future Improvements
* Implement Machine Learning (TinyML) for more accurate audio classification.
* Battery optimization and deep sleep modes.
* Add a vision to the system using a cam.
