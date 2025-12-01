# Multi-Sensor Gait & Activity Data Logger: Training Platform for HAR TinyML

## 💡 Project Overview 

This project repurposes the ESP32 as a wearable, self-contained data acquisition platform. The main goal is to generate large, labelled datasets of human walking activity under real-world conditions.

The collected data will be a multivariate time series suitable for training classification models (e.g., CNNs or LSTMs) to distinguish between different walking styles, speeds, or activities (e.g., walking, running, ascending hills or stairs).

---

## Key Components

### **ICM20948 (IMU)**
- Provides high-frequency (e.g., 50 Hz) inertial measurements (acceleration, rotation, magnetic field).
- Captures the dynamics of human movement.

### **GPS6MV2**
- Provides location, velocity, and altitude data.
- Enables correlation of movement patterns with environmental context (e.g., changes in terrain, gradients, or speed).

### **Micro SD Card**
- Stores gigabytes of high-frequency data generated during long-duration walks.

### **OLED Display**
- Shows real-time feedback: logging status, GPS fix quality, battery level.

---

## 🛠️ Hardware Setup and Wiring

The ESP32 requires careful wiring to handle two I²C devices (IMU, OLED) and a serial device (GPS).

| Component              | ESP32 Pin          | Interface        | Notes                                                |
|-----------------------|---------------------|------------------|------------------------------------------------------|
| ICM20948 (IMU)        | SDA/SCL (GPIO 21/22)| I²C              | Standard I²C bus for the IMU.                        |
| OLED Display (128×64) | SDA/SCL (GPIO 21/22)| I²C              | Shares the I²C bus with the IMU.                     |
| GPS6MV2 (TX)          | RXD2 (GPIO 16)      | UART (Serial2)   | Reliable, high-speed GPS input.                      |
| GPS6MV2 (RX)          | TXD2 (GPIO 17)      | UART (Serial2)   | Optional: used for sending configuration commands.   |
| SD Card Reader (CS)   | GPIO 5              | SPI (Custom CS)  | Uses default SPI pins (SCK, MOSI, MISO).             |

---

## 💾 Data Logging Format (CSV Output)

Each line written to the SD card is a timestamped observation, ready for import into Python/Pandas. The filename is dynamically generated (e.g., `LOG_YYMMDD.CSV`).

| Field           | Unit  | Sensor     | Description                                      |
|----------------|--------|------------|--------------------------------------------------|
| Time_ms        | ms     | millis()   | Time since startup.                              |
| Accel_X,Y,Z    | m/s²   | ICM20948   | Linear acceleration.                             |
| Gyro_X,Y,Z     | deg/s  | ICM20948   | Angular velocity.                                |
| Mag_X,Y,Z      | µT     | ICM20948   | Magnetic field strength.                         |
| Latitude       | deg    | GPS6MV2    | Current latitude.                                |
| Longitude      | deg    | GPS6MV2    | Current longitude.                               |
| Speed          | knots  | GPS6MV2    | Speed over ground.                               |
| Altitude       | m      | GPS6MV2    | Altitude above sea level.                        |
| Satellites     | count  | GPS6MV2    | GPS fix quality.                                 |

---

## 📈 Training Goal

Primary ML task: **Classification**

Given a one-second window of engineered features, classify the activity into one of the following types:

- Standing still  
- Level walking (slow)  
- Level walking (normal/brisk)  
- Uphill/stairs ascending  
- Running  
- Transitional/resting (stopping, starting, sitting down, standing up)  
- Anomalous gait (limping, carrying a heavy load, shuffling)

These may be replaced or augmented by the expanded GPS-only categories listed below.

---

## ⚙️ Post-Processing and Feature Engineering

Raw 15-channel time-series data is processed in Python/Pandas to create an orientation-invariant, reliably labelled dataset.

---

### 1. Orientation Normalisation (9-DOF Sensor Fusion)

To compensate for varying device placement, the 9-DOF data is transformed from the sensor’s frame into the Earth frame (North, East, Down).

**Tool:**  
Madgwick or Extended Kalman Filter (EKF)

Given raw `Accel_*`, `Gyro_*`, and `Mag_*`, the filter outputs a **quaternion** describing orientation.

**Result:**  
Compute **vertical acceleration** independent of device tilt — the key feature for gait cycle detection.

---

### 2. Derived Feature Generation

Derived features are less noisy and more orientation-stable than raw IMU signals.

| Feature Name                      | Calculation                                                             | Rationale |
|----------------------------------|--------------------------------------------------------------------------|-----------|
| **Total Acceleration Magnitude (TAM)** | `sqrt(Accel_X² + Accel_Y² + Accel_Z²)`                                  | Detect any movement. |
| **Gyroscope Magnitude (GM)**         | `sqrt(Gyro_X² + Gyro_Y² + Gyro_Z²)`                                     | Identify turns, pivots, or stumbles. |
| **Vertical Acceleration**            | From sensor fusion quaternion                                           | Captures the up-and-down oscillation of gait. |
| **Magnetic Field Magnitude**         | `sqrt(Mag_X² + Mag_Y² + Mag_Z²)`                                        | Assess magnetic interference. |

---

### 3. Data Labelling Strategy

Assign ground-truth activity classes using GPS and IMU data:

#### **Speed-Based Classification**
Use GPS speed thresholds to automatically label:
- Standing  
- Walking  
- Running  

#### **Altitude-Based Classification**
Use rate of change:  
`d(Altitude) / dt`  
→ Identify stair climbing or uphill walking.

#### **Contextual Labelling**
Use bounding boxes (geofencing) for known environments to label activities with high certainty.

---

# 🗂️ Refined & More Interesting GPS-Derived Activity Categories

Below is an expanded set of activity categories designed to be labelled automatically using **GPS-only data**. These categories introduce more variety and contextual nuance while remaining suitable for automated, high-confidence annotation using speed, altitude rate, heading changes, and geofencing.

---

## 🚶 Standard Gait & Mobility Categories (GPS Only)

### **1. Stationary**
- **GPS condition:** Speed < 0.3 knots for ≥ 5 seconds  
- Includes standing still, resting, or waiting.

### **2. Slow Walking**
- **GPS condition:** 0.3–2.3 knots  
- Typical of cautious or leisurely movement.

### **3. Normal Walking**
- **GPS condition:** 2.3–4.0 knots  
- Baseline, comfortable gait speed.

### **4. Fast / Brisk Walking**
- **GPS condition:** 4.0–6.0 knots  
- Often seen in purposeful or fitness-oriented walking.

### **5. Running / Jogging**
- **GPS condition:** > 6 knots  
- Easily separable based on speed.

---

## 🧗 Terrain & Gradient-Related Categories

Uses altitude change rate (dAltitude/dt):

### **6. Uphill Walking**
- **GPS condition:**  
  - Speed < 6 knots  
  - dAltitude/dt > +0.5 m/s  

### **7. Downhill Walking**
- **GPS condition:**  
  - Speed < 6 knots  
  - dAltitude/dt < –0.5 m/s  

### **8. Stair Climbing / Rapid Elevation Change**
- **GPS condition:**  
  - Sharp altitude increases (e.g., 2–4 m in < 10 s)  
- Best labelled via geofencing due to GPS altitude noise.

---

## 🧭 Contextual & Path-Shape Categories (GPS Geometry)

### **9. Zig-Zag / Curved Path Walking**
- **GPS condition:** Frequent heading changes > 30°  
- Represents obstacle avoidance or crowded environments.

### **10. Straight-Line Walking**
- **GPS condition:** Heading variance < 5°  
- Ideal for controlled gait studies.

### **11. Stop–Start Transitional Walking**
- **GPS condition:** Frequent switching between stationary and slow walking  
- Captures gait initiation and termination.

---

## 🎒 Activity Context Categories (GPS Region or Behaviour)

### **12. Indoor vs Outdoor Walking**
- Indoor GPS characteristics:  
  - Reduced precision  
  - Fewer satellites  
  - Erratic speed readings  
- Label via geofencing.

### **13. Urban Walking (Dense Environment)**
- Frequent speed interruptions  
- Higher satellite drop-outs  
- Numerous direction changes  
- Label by mapping urban polygons.

### **14. Open-Field / Park Walking**
- Longer straight segments  
- More stable speed  
- Lower heading variance  
- Geofence outdoor areas.

---

## 🧳 Real-World Gait Challenge Categories

### **15. Walking With Load**
- **GPS proxies:**  
  - Reduced comfortable walking speed  
  - Lower speed variability  
- Best combined with route-based labels.

### **16. Route-Following Navigation**
- **GPS condition:** Path matches a predefined GPX route.  

### **17. Random Exploration**
- **GPS condition:** High heading variance, no consistent direction.

---

## ⭐ Recommended Final Category Set (GPS Labelled)

A balanced and varied set ideal for training robust HAR/TinyML models:

1. **Stationary**  
2. **Slow Walking**  
3. **Normal Walking**  
4. **Fast / Brisk Walking**  
5. **Running / Jogging**  
6. **Uphill Walking**  
7. **Downhill Walking**  
8. **Straight-Line Walking**  
9. **Zig-Zag / Curved Path Walking**  
10. **Stop–Start Transitional Walking**  
11. **Urban Walking**  
12. **Open-Field / Park Walking**

This set provides rich variation in:
- speed  
- terrain gradient  
- movement geometry  
- environmental context  

ensuring high-quality, generalisable HAR and gait classification models.


