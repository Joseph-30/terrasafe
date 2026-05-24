---
publishDate: 2026-05-24T00:00:00Z
title:" Bouynet - Terrasafe 
A ESP32 9-DOF Breadcrumb Trail Navigator"

excerpt: An advanced, GPS-free dead-reckoning navigation system using a 9-DOF sensor array to track movement and guide users back to their origin.
image: esp32-9dof-cover.jpg
tags:
  - ESP32
  - Sensor-Fusion
  - IoT
---
> An advanced, GPS-free dead-reckoning navigation system built on the ESP32.

---
## Acknowledgements

Thank you to the creators of the Adafruit and QMC5883LCompass libraries for providing robust sensor integration foundations for this project. Thank you to the Sensors Council for organizing the MYOSA Event!

---
## Overview
The ESP32 9-DOF Breadcrumb Trail Navigator is a hardware and software solution that allows users to navigate and track their movements without relying on GPS signals. 

* **What the project does:** It records a user's path using dead-reckoning and provides a targeted, visual UI to guide them backward, step-by-step, to their origin. It also hosts an interactive web dashboard for live telemetry.
* **How it works:** It utilizes a 9-DOF sensor array (Accelerometer, Gyroscope, Magnetometer) paired with a barometer. Raw data is processed through complementary and Kalman filters for stable heading, while autocorrelation algorithms detect walking steps to map X/Y/Z coordinates.
* **Who it is for:** Hikers, cave explorers, and rescue personnel operating in environments where GPS signals are weak, jammed, or non-existent.
* **What problem it solves:** Standard GPS devices fail underground, deep inside concrete structures, or in dense canopies. This system solves the problem of getting lost in these environments by relying entirely on local, self-contained sensor fusion for return navigation.

---
## Demo / Examples

### **Images**
<p align="center">
<img src="esp32-9dof-cover.jpg" width="800"><br/>
<i>The ESP32 Navigator device hardware setup</i>
</p>

<p align="center">
<img src="web_interface_image.png" width="800"><br/>
<i>The responsive, dark-mode web dashboard hosted on the ESP32</i>
</p>

### **Videos**


<video controls width="100%">
  <source src="myosa_demo_video.mp4" type="video/mp4">
</video>

*(Note: If the video player above does not load in GitHub, use the link below)*
[🎥 Click here to download/watch the Demo Video (Raw File)](myosa_demo_video.mp4)

<video controls width="100%">
<source src="myosa_presentation_video.mp4" type="video/mp4">
</video>

*(Note: If the video player above does not load in GitHub, use the link below)*
[🎥 Click here to download/watch the Presentation Video (Raw File)](myosa_presentation_video.mp4)

---
## Features (Detailed)

### **1. 9-DOF Sensor Fusion & Kalman Filtering**
Combines the MPU6050 (Accel/Gyro) and QMC5883L (Magnetometer). Raw coordinates calculated from step distance and heading are passed through discrete Kalman filters (`kalman_x`, `kalman_y`) to smooth out jitter, sensor noise, and provide tilt-compensated orientation (Pitch, Roll, Yaw).

### **2. Robust Autocorrelation Pedometer**
Instead of simple peak-detection, accelerometer data is stored in a ring buffer and evaluated using autocorrelation. This determines true human walking cadence, actively filtering out false movements like vehicle bumps or hand waving.

### **3. Vertical Tracking System**
Integrates a BMP180 barometer to track absolute altitude and relative depth/height. It automatically estimates building floor levels or cave depths, though it remains sensitive to long-term weather-induced pressure changes.

### **4. Breadcrumb Reverse Navigation & Waypoints**
Automatically records the user's path. Drop categorized waypoints (Camp, Water, Danger) and receive proximity alerts. With a single button click, the system provides a visual UI to guide you backward to your starting point.

### **5. Embedded Web Server & SOS Beacon**
Hosts a responsive HTML/JS/CSS dashboard directly on the ESP32 (no SD card required). It provides live telemetry, interactive canvas mapping, vertical tracking charts, and allows remote triggering of an emergency SOS visual beacon.

---
## Usage Instructions

### 1. Startup Calibration (Mandatory)
When powered on, the device enters a mandatory calibration phase to ensure dead-reckoning accuracy.
* **Gyroscope Calibration:** Keep the device perfectly still on a flat surface.
* **Compass Calibration:** Follow the on-screen prompts (Figure-8 motions, vertical rotations, and tilts) for 15 seconds.

### 2. Physical Interface (The Button)
The push button (GPIO 16) controls the core navigation states:
* **Single Click:** Toggles between Forward Mode (recording path) and Reverse Mode (following breadcrumbs back to start).
* **Double Click:** Activates the 3D Dashboard on the OLED.

### 3. The Web Interface
1. Connect your phone or laptop to the Wi-Fi network: **`myosa`** (No password by default).
2. Open a web browser and navigate to `http://192.168.4.1`.

---
## Tech Stack
* **Microcontroller:** ESP32 Development Board
* **Sensors:** MPU6050 (IMU), QMC5883L (Magnetometer), BMP180 (Barometer)
* **Display:** 0.96" I2C OLED (SSD1306, 128x64)
* **Software:** Arduino Framework, C++, HTML/CSS/JS (Web Server)

---
## Requirements / Installation

### Hardware Wiring
All sensors share the primary I2C bus (3.3V Logic).
* **I2C SDA:** GPIO 21
* **I2C SCL:** GPIO 22
* **Reverse Button:** GPIO 16 (Connect to GND, uses internal pull-up)

### Software Setup
1. Clone the repository and open the `.ino` file in the Arduino IDE.
2. Update the `MAGNETIC_DECLINATION` in the code for your specific geographic location (currently `-0.72` for Thrissur).
3. Install the following dependencies via the Arduino Library Manager:

```plaintext
Adafruit MPU6050
Adafruit BMP085 Library
Adafruit SSD1306
Adafruit GFX Library
Adafruit Unified Sensor
QMC5883LCompass (by mprograms)
ArduinoJson
```
## File Structure (Optional)
```

/terrasafe
├─ readme.md
├─ index.ino
├─ esp32-9dof-cover.jpg
├─ web_interface_image.png
├─ myosa_demo_video.mp4
├─ myosa_presentation_video.mp4
└─ myosa_presentation_video.pdf
```
