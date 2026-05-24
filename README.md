# 🧭 ESP32 9-DOF Breadcrumb Trail Navigator (V5)

![Version](https://img.shields.io/badge/Version-5.0-blue.svg)
![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg)
![Framework](https://img.shields.io/badge/Framework-Arduino-blue.svg)

An advanced, GPS-free dead-reckoning navigation system built on the ESP32. This project utilizes a 9-DOF sensor array (Accelerometer, Gyroscope, Magnetometer, and Barometer) paired with Kalman filtering and autocorrelation algorithms to track movement, map paths, and guide users back to their starting point using visual breadcrumbs. 

It features an onboard OLED display for real-time tracking and a rich, interactive web dashboard hosted directly on the ESP32.

---

## ✨ Key Features

* **🧠 9-DOF Sensor Fusion:** Combines MPU6050 (Accel/Gyro) and QMC5883L (Magnetometer) with complementary and Kalman filters for stable, tilt-compensated heading and orientation (Pitch, Roll, Yaw).
* **🚶 Robust Pedometer:** Uses autocorrelation algorithms to detect steps and calculate cadence, filtering out false movements.
* **🏔️ Vertical Tracking System:** Integrates a BMP180 barometer to track absolute altitude, relative depth/height, and automatically estimate building floor levels or cave depths.
* **🗺️ Breadcrumb Reverse Navigation:** Automatically records your path and provides a targeted, visual UI to guide you backward step-by-step to your origin.
* **📍 Smart Waypoint Management:** Drop categorized waypoints (Camp, Water, Danger, etc.) and receive proximity alerts on the OLED and Web UI.
* **🆘 Emergency SOS System:** Triggers an emergency visual beacon and sets an SOS flag for simulated BLE/Radio transmission.
* **🌐 Embedded Web Server:** Hosts a responsive, dark-mode-supported HTML/JS/CSS dashboard providing live telemetry, interactive canvas mapping, and vertical tracking charts.

---

## 🛠️ Hardware Requirements

* **Microcontroller:** ESP32 Development Board
* **IMU (Accel/Gyro):** MPU6050
* **Magnetometer:** QMC5883L
* **Barometer:** BMP180 (or BMP085)
* **Display:** 0.96" I2C OLED (SSD1306, 128x64)
* **Input:** Push Button (Normally Open)
* **Misc:** Breadboard, jumper wires, pull-up resistors (if not included on sensor breakout boards).

### Pinout & Wiring

All sensors share the primary I2C bus.

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **I2C SDA** | GPIO 21 | Shared by MPU6050, QMC5883L, BMP180, OLED |
| **I2C SCL** | GPIO 22 | Shared by MPU6050, QMC5883L, BMP180, OLED |
| **VCC (Sensors)** | 3.3V | Ensure your sensors are 3.3V compatible |
| **GND** | GND | Common Ground |
| **Reverse Button** | GPIO 16 | Connect other leg to GND (Uses internal pull-up) |

---

## 📚 Software Dependencies

Install the following libraries via the Arduino IDE Library Manager:

* `Adafruit MPU6050`
* `Adafruit BMP085 Library`
* `Adafruit SSD1306`
* `Adafruit GFX Library`
* `Adafruit Unified Sensor`
* `QMC5883LCompass` (by mprograms)
* `ArduinoJson` (v6.x or v7.x)

---

## 🚀 Installation & Setup

1. **Clone the Repository:**
```bash
   git clone [https://github.com/yourusername/esp32-9dof-navigator.git](https://github.com/yourusername/esp32-9dof-navigator.git)
```
1. **Open the Project:** 
   Open the `.ino` file in the Arduino IDE (or PlatformIO).
2. **Configure Settings:**
   * Open the code and locate `MAGNETIC_DECLINATION`. Update it for your specific geographic location (currently set for Thrissur, Kerala: `-0.72`).
   * Modify the Wi-Fi AP credentials if desired (`ap_ssid` and `ap_password`).
3. **Compile and Upload:**
   Select your specific ESP32 board and COM port, then click **Upload**.

---

## 🎮 Usage & Calibration

### 1. Startup Calibration (Mandatory)
When powered on, the device enters a mandatory calibration phase to ensure dead-reckoning accuracy.
* **Gyroscope Calibration:** Keep the device perfectly still on a flat surface.
* **Compass Calibration:** Follow the on-screen prompts (Figure-8 motions, vertical rotations, and tilts) for 15 seconds.

### 2. Physical Interface (The Button)
The push button (GPIO 16) controls the core navigation states:
* **Single Click:** Toggles between **Forward Mode** (recording path) and **Reverse Mode** (following breadcrumbs back to start).
* **Double Click:** Activates the **3D Dashboard** on the OLED. The dashboard cycles through a 4-quadrant sensor readout and an Altitude/Vertical monitoring screen before returning to the map.

### 3. The Web Interface
The ESP32 creates its own Wi-Fi Access Point so you can connect in the wilderness without a router.
1. Connect your phone or laptop to the Wi-Fi network: **`myosa`** (No password by default).
2. Open a web browser and navigate to `http://192.168.4.1`.

**Features of the Web App:**
* Live interactive path mapping with pinch-to-zoom and pan.
* Add, categorize, and delete smart waypoints.
* View live graphs of your vertical ascent/descent.
* Trigger SOS mode remotely.
* Toggle Reverse Navigation mode.

---

## 🏗️ System Architecture Highlights

* **Kalman Filtering:** Raw coordinates calculated from step distance and heading are passed through discrete Kalman filters (`kalman_x`, `kalman_y`) to smooth out jitter and sensor noise.
* **Autocorrelation Pedometer:** Instead of simple peak-detection, the accelerometer data is stored in a ring buffer and evaluated using autocorrelation to determine true human walking cadence (ignoring vehicle bumps or hand waving).
* **Asynchronous Web Server:** The HTML, CSS, and JS are bundled directly into the ESP32 code as raw literals, meaning no external SD card is required to host the complex dashboard.

---

## ⚠️ Limitations & Notes

* **Drift:** Dead-reckoning relies on IMUs. Over long distances (kilometers), positional drift is inevitable due to sensor noise accumulation. This system is designed for short-to-medium range exploration (e.g., hiking trails, caves, large buildings).
* **Barometer Sensitivity:** The BMP180 is sensitive to weather-induced pressure changes. If you hike for many hours, natural atmospheric pressure shifts may cause slight altitude drift.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

