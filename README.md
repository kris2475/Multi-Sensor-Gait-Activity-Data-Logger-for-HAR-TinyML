# Project Overview
### **High-Fidelity Human Activity + Motion Context Recognition Using ESP32 + GPS + IMU + Magnetometer**

This project transforms a compact ESP32-based multi-sensor device into a *fully autonomous human activity classifier*, capable of recording, understanding, and **labelling** motion in real-world environments — **without any manual annotation**.

The system fuses **GPS (Neo-6M)**, **inertial data (MPU6050)**, and **magnetometer heading (QMC5883L)** into a rich activity taxonomy that spans human gait, terrain context, and even the new **Driving / Vehicle Motion** category.

It is designed to feel like a professional research instrument, yet still fits into a pocket-sized ESP32 development board.

---

# Hardware Architecture

### **1. ESP32 Dev Module**
- Central processing unit  
- **Decoupled, synchronised sampling** logic to manage different sensor rates  
- Handles I2C, UART GPS, VSPI SD logging, and OLED rendering  
- Enough horsepower for onboard feature extraction if desired

### **2. GPS Module — Neo-6M (UART2 @ 9600)**
- Provides: latitude, longitude, altitude, heading, speed over ground, satellite count  
- **GATED LOGGING:** Logging is automatically **disabled** unless a minimum number of satellites (e.g., $\ge 5$) are acquired, preventing worthless log entries.
- Critical for detecting:
  - Straight vs curved paths  
  - Stop–start behaviour  
  - Urban vs open-field classification  
  - Vehicle-level speeds for **Driving**

### **3. IMU — MPU6050 (Accel + Gyro, I2C 0x68)**
Reading raw registers directly:
- **Accel ($\mathbf{ax, ay, az}$)** @ $\pm 8\text{g}$  
- **Gyro ($\mathbf{gx, gy, gz}$)** @ $\pm 500^\circ\text{/s}$  
- **Sampled at a high rate (e.g., $\mathbf{50 \text{ Hz}}$)** to capture fine gait cycles.

Used for:
- Step detection  
- Gait periodicity  
- Running vs walking transitions  
- Determining stationary state  
- Detecting effort on slopes  
- Recognising absence of intra-body motion inside a car

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
- **GPS Fix Status (GATING feedback)** $\leftarrow$ *New*
- GPS lock & sats  
- Accelerometer updates  
- Gyro drift  
- Magnetometer readings  
- SD logging status

### **6. Micro SD Card (VSPI)**
- Logs all sensor streams in $\mathbf{two \text{ synchronised CSV files}}$  
- Logging is **buffer-managed** and only occurs when a GPS fix is established.  
- Forms the dataset for offline model training

---

# Data Logging Architecture: Decoupled Sampling

To maximise data richness while ensuring high-quality **labels**, the system uses a **Decoupled Sampling Architecture** resulting in two synchronised log files linked by the $\mathbf{\text{SampleID}}$.

### **File 1: `/sensor.csv` (High-Frequency Input Data)**
- **Rate:** **50 Hz** (a new row every **20 ms**).
- **Purpose:** Input features for Deep Learning models.

| Field | Description |
|-------|-------------|
| **SampleID** | Continuous, running counter (**0, 1, 2, ...**). Used as the temporal key. |
| AccX/Y/Z | Linear acceleration ($\mathrm{m/s}^2$) |
| GyroX/Y/Z | Angular velocity ($\text{deg/s}$) |
| MagX/Y/Z | Magnetic field vector |

### **File 2: `/gps_label.csv` (Low-Frequency Ground-Truth Labels)**
- **Rate:** **1 Hz** (a new row every **1 second**).
- **Purpose:** Ground-truth **labels** for the $\approx 50$ sensor samples in the corresponding time window.

| Field | Description |
|-------|-------------|
| **StartSampleID** | The **SampleID** where the 1-second GPS window *begins*. Used as the synchronisation key. |
| Lat/Lon | GPS coordinate |
| Alt | Altitude in metres |
| Sats | Satellite count (Guaranteed $\ge 5$ due to fix gate) |
| Speed_kph | Speed over ground in $\mathrm{km/h}$ |

This dataset is rich enough to drive:
- Human activity recognition  
- **Motion-to-speed regression** (predicting **Speed\_kph** from IMU data)  
- Vehicle vs pedestrian separation  
- Map-matching or path reconstruction  
- Heading drift correction  

---

# Activity Taxonomy  
## **Designed for Automatic Labelling (No manual annotation required)**

### **1. Base Locomotion Categories**

### **Stationary**
- No step periodicity  
- Accel & gyro variance collapse  
- Magnetometer stable  
- GPS speed < 0.5 knots  

### **Slow Walking**
- GPS $\approx 1–2 \text{ knots}$  
- Low amplitude gait oscillation  
- Small heading jitter

### **Normal Walking**
- GPS $\approx 2–3 \text{ knots}$  
- Clean Z-axis periodicity  
- Gyro lateral rotation patterns  
- Magnetometer micro-sway detectable

### **Brisk Walking**
- GPS $\approx 3–4.5 \text{ knots}$  
- Higher cadence  
- Stronger impacts  
- Faster heading progression

### **Running**
- GPS $4.5–7 \text{ knots}$  
- Large IMU variance  
- Strong harmonic step frequency  
- Pronounced gyro roll

---

### **2. Terrain & Directional Categories**

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
- Speed oscillates between 0 and $>1 \text{ knot}$  
- Burst motions + still periods  

---

### **3. Contextual Walking Categories**

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

### **4. Driving / Vehicle Motion (New)**

### **Driving**
- GPS speed $> 7 \text{ knots}$  
- Very low IMU amplitude  
- No step-cycle harmonic  
- Magnetometer heading smooth, slow curvature  
- Gyro low except during turns  
- Zero body micro-oscillation $\rightarrow$ primary indicator  

Driving is **shockingly easy** to detect with your sensor set — much easier than distinguishing slow walking from terrain changes.

It adds depth to your dataset and allows richer context recognition for real-world movement.

---

# Why This System Works So Well: Deep Learning Ready

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

This dataset is built to be a direct fit for the most powerful deep learning models:

### **ML Dataset Transformation: Windowing & 3D Array**

The key to using this data is transforming it from the two CSV files into a $\mathbf{3D \text{ NumPy array}}$ required by Deep Learning models.

1.  **Windowing:** The $\mathbf{\text{/sensor.csv}}$ data is segmented into non-overlapping $\mathbf{\approx 1 \text{ second windows}}$ (e.g., $50 \text{ samples} \times 9 \text{ features}$).
2.  **Label Alignment:** The single corresponding row from $\mathbf{\text{/gps\_label.csv}}$ is used to **label** the entire $\approx 50$ samples within that window (e.g., $\mathbf{\text{Speed\_kph} = 5.5}$).
3.  **Final Shape:** The input data is structured as $\mathbf{(\text{Number of Windows}, \text{Time Steps}, \text{Features})}$, creating a $3\text{D}$ tensor.

### **Suitability for CNN and LSTM Architectures**

| Architecture | Role | Why This Data Fits |
| :--- | :--- | :--- |
| **CNN** (Convolutional Neural Network) | **Feature Extractor** | CNNs can treat the $50 \times 9$ time window as a signal segment, automatically learning **local, translation-invariant patterns** like the precise shape of a heel strike, which simple statistics miss. |
| **LSTM** (Long Short-Term Memory) | **Sequence Modeller** | LSTMs excel at modelling **temporal dependencies** within the 50-sample sequence, capturing how the motion evolves over the full second (e.g., smooth acceleration vs. abrupt halt). |

By combining the strengths of CNNs for feature extraction and LSTMs for sequence modelling (often in a **ConvLSTM** or similar architecture), you can achieve high accuracy in predicting continuous values like $\mathbf{\text{Speed\_kph}}$ or classifying fine-grained activities like **Uphill vs. Downhill Walking**.

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
- Add BLE beacon scanning for indoor localisation  

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
- Automatically **labellable**  
- Rich enough for research-grade machine learning  
- Interesting enough to tell a compelling story

