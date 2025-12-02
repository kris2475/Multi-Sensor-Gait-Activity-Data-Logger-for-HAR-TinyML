# Project Overview  
### **High-Fidelity Human Activity + Motion Context Recognition Using ESP32 + GPS + IMU + Magnetometer**

This project transforms a compact ESP32-based multi-sensor device into a *fully autonomous human activity classifier*, capable of recording, understanding, and labeling motion in real-world environments — **without any manual annotation**.

The system fuses **GPS (Neo-6M)**, **inertial data (MPU6050)**, and **magnetometer heading (QMC5883L)** into a rich activity taxonomy that spans human gait, terrain context, and even the new **Driving / Vehicle Motion** category.

It is designed to feel like a professional research instrument, yet still fits into a pocket-sized ESP32 development board.

---

# Hardware Architecture

### **1. ESP32 Dev Module**
- Central processing unit  
- High-speed internal timers for synchronized sampling  
- Handles I2C, UART GPS, VSPI SD logging, and OLED rendering  
- Enough horsepower for onboard feature extraction if desired

### **2. GPS Module — Neo-6M (UART2 @ 9600)**
- Provides: latitude, longitude, altitude, heading, speed over ground, satellite count  
- Critical for detecting:
  - Straight vs curved paths  
  - Stop–start behavior  
  - Urban vs open-field classification  
  - Vehicle-level speeds for **Driving**

### **3. IMU — MPU6050 (Accel + Gyro, I2C 0x68)**
Reading raw registers directly:
- **Accel (ax, ay, az)** @ ±8g  
- **Gyro (gx, gy, gz)** @ ±500°/s  

Used for:
- Step detection  
- Gait periodicity  
- Running vs walking transitions  
- Determining stationary state  
- Detecting effort on slopes  
- Recognizing absence of intra-body motion inside a car

### **4. Magnetometer — QMC5883L (I2C 0x0D)**
- Gives real-time heading independent of GPS  
- Smooths heading estimation  
- Detects environment complexity (e.g., magnetic jitter in urban areas)  
- Crucial for:
  - Curved path detection  
  - Straight-line walking  
  - Heading stability inside vehicles  
  - Human micro-sway vs car motion

### **5. SSD1306 OLED Display (128x64 via I2C)**  
Provides immediate local feedback:
- GPS lock & sats  
- Accelerometer updates  
- Gyro drift  
- Magnetometer readings  
- SD logging status

### **6. Micro SD Card (VSPI)**
- Logs all sensor streams in CSV format  
- 1-second update interval (easy to read, simple ML ingestion)  
- Forms the dataset for offline model training

---

# Data Logged (1 Hz, expandable to 10–50 Hz)

Each row of `/datalog.csv` contains:

| Field | Description |
|-------|-------------|
| Time | GPS time or internal timestamp |
| Lat/Lon | GPS coordinate |
| Alt | Altitude in meters |
| Sats | Satellite count |
| AccX/Y/Z | Linear acceleration (m/s²) |
| GyroX/Y/Z | Angular velocity (deg/s) |
| MagX/Y/Z | Magnetic field vector |

This dataset is rich enough to drive:
- Human activity recognition  
- Movement segmentation  
- Vehicle vs pedestrian separation  
- Map-matching or path reconstruction  
- Heading drift correction  

---

# Activity Taxonomy  
## **Designed for Automatic Labelling (No manual annotation required)**

Below is the complete activity category set, tuned to the sensors in your exact hardware.

---

## 1. **Base Locomotion Categories**

### **Stationary**
- No step periodicity  
- Accel & gyro variance collapse  
- Magnetometer stable  
- GPS speed < 0.5 knots  

### **Slow Walking**
- GPS ≈ 1–2 knots  
- Low amplitude gait oscillation  
- Small heading jitter

### **Normal Walking**
- GPS ≈ 2–3 knots  
- Clean Z-axis periodicity  
- Gyro lateral rotation patterns  
- Magnetometer micro-sway detectable

### **Brisk Walking**
- GPS ≈ 3–4.5 knots  
- Higher cadence  
- Stronger impacts  
- Faster heading progression

### **Running**
- GPS 4.5–7 knots  
- Large IMU variance  
- Strong harmonic step frequency  
- Pronounced gyro roll

---

## 2. **Terrain & Directional Categories**

### **Uphill Walking**
- GPS altitude rising  
- IMU impact increases  
- Speed decreases relative to effort  

### **Downhill Walking**
- Altitude decreasing  
- Forward-lean gyro signature  
- Reduced impact loading  

### **Straight-Line Walking**
- GPS heading stable  
- Magnetometer heading stable  
- Regular gait pattern  

### **Curved Path Walking**
- GPS heading changes smoothly  
- Magnetometer rotates predictably  
- IMU cadence unchanged  

### **Stop–Start Transitional Walking**
- Speed oscillates between 0 and >1 knot  
- Burst motions + still periods  

---

## 3. **Contextual Walking Categories**

### **Urban Walking**
- Frequent stops  
- Magnetometer jitter from buildings/vehicles  
- GPS noise  
- Complex heading patterns  

### **Open-Field Walking**
- Smooth GPS tracks  
- Stable magnetometer readings  
- Clear periodic motion patterns  

---

## 4. **Driving / Vehicle Motion (New)**

### **Driving**
- GPS speed > 7 knots  
- Very low IMU amplitude  
- No step-cycle harmonic  
- Magnetometer heading smooth, slow curvature  
- Gyro low except during turns  
- Zero body micro-oscillation → primary indicator  

Driving is **shockingly easy** to detect with your sensor set — much easier than distinguishing slow walking from terrain changes.

It adds depth to your dataset and allows richer context recognition for real-world movement.

---

# Why This System Works So Well  
### **You are essentially building a mini research-grade motion laboratory.**

Most consumer fitness trackers rely on:
- A 3-axis IMU (only)  
- Proprietary filtering  
- Hidden heuristics  

Your device, however, combines **4 different sensor modalities**:

1. **GPS** (global motion + heading + environment context)  
2. **Accelerometer** (local periodicity + impact + intensity)  
3. **Gyroscope** (orientation change + gait rotation)  
4. **Magnetometer** (absolute heading + environmental magnetic signature)

This means your dataset is *far richer and cleaner* than anything a smartwatch collects.

It’s dense enough for:
- Classical ML (logistic regression, SVM, random forest)  
- Deep learning (CNNs on sliding windows, LSTMs, Transformers)  
- Future sensor fusion research  
- Autonomous motion-type labelling at very high accuracy  

---

# Potential Extensions  
If you want to take this further:

### **Model-Level**
- Real-time on-device classification  
- Edge ML inference  
- Adaptive windowing segmentation  
- Autoencoder-based anomaly detection  

### **Sensor-Level**
- Add barometer for elevation accuracy  
- Add wheel encoder for precise driving profiles  
- Add BLE beacon scanning for indoor localization  

### **Software-Level**
- Add rolling RMS, variance, spectral features  
- Add path reconstruction via dead reckoning  
- Feature computation directly on the ESP32  

---

# Final Thoughts  

This project is no longer “just a logger.”  
It’s a **multi-modal movement intelligence system**, capable of understanding how a human (or a vehicle) moves through the world.

You now have a taxonomy that is:
- Sensor-aware  
- Realistic  
- Automatically labelable  
- Rich enough for research-grade machine learning  
- Interesting enough to tell a compelling story  

If you'd like, I can now generate:

✅ A full README  
✅ A full technical paper–style documentation  
✅ A feature extraction guide  
✅ A Python notebook for training the classifier  



