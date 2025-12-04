/*
 * ESP32 Multi-Sensor Logger with Decoupled Sampling Rates for ML Dataset
 * - HIGH-FREQUENCY SENSOR SAMPLING and LOGGING IS GATED by a GPS Fix.
 * * Sensors:
 * - GPS (Neo-6M on UART2)
 * - MPU6050 (Accel/Gyro, I2C 0x68)
 * - QMC5883L (Magnetometer, I2C 0x0D)
 * Display:
 * - SSD1306 OLED (I2C 0x3C)
 * SD:
 * - MicroSD via VSPI (CS 5)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// ---------------- CONFIG ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SD_CS_PIN 5
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
#define BUTTON_PIN 27 // Push button on D27

// --- GPS GATE CONFIG ---
// Minimum number of satellites required to start logging. 
// 4 is the minimum for a 3D fix, but 5+ is recommended for accuracy.
#define MIN_SATS_FOR_LOGGING 5 

// --- SAMPLING CONFIG ---
#define SENSOR_SAMPLE_MS 20      // 50 Hz (1000ms / 20ms)
#define GPS_SAMPLE_MS 1000     // 1 Hz
#define SENSOR_BUFFER_SIZE 1500  // Holds 30 seconds of 50Hz data (30 * 50)
#define GPS_BUFFER_SIZE 30     // Holds 30 seconds of 1Hz data (30 * 1)
#define PAGE_INTERVAL 1000       // ms

// ---------------- OBJECTS ----------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ---------------- SENSOR BUFFERS ----------------

// High-Frequency Sensor Data structure
struct SensorSample {
    uint32_t sampleID; 
    float ax, ay, az;
    float gx, gy, gz;
    int16_t mx, my, mz;
};

// Low-Frequency GPS Data structure
struct GpsSample {
    uint32_t startSampleID; 
    double lat, lng;
    double alt;
    double speed; // kph
    uint8_t sats;
    uint8_t hour, minute, second;
};

// Buffers
SensorSample sensorBuffer[SENSOR_BUFFER_SIZE];
GpsSample gpsBuffer[GPS_BUFFER_SIZE];

uint16_t sensorBufferIndex = 0; 
uint8_t gpsBufferIndex = 0;     
uint32_t currentSampleID = 0;   
bool sdAvailable = false;
uint32_t totalFlushes = 0; 
bool isLoggingActive = false; // NEW FLAG

// ---------------- OTHER ----------------
unsigned long lastSensorSampleTime = 0; 
unsigned long lastGpsSampleTime = 0;    
uint8_t oledPage = 0;
unsigned long lastPageSwitch = 0;

// ---------------- FORWARD DECLARATION ----------------
void flushBuffersToSD();

// ---------------- MPU6050 ----------------
class MPU6050_Raw {
public:
    void begin() {
        Serial.println("Initializing MPU6050...");
        Wire.beginTransmission(0x68);
        Wire.write(0x6B); Wire.write(0x00); 
        if (Wire.endTransmission(true) != 0) { 
            Serial.println("MPU6050 WAKEUP FAILED - Check address/wiring!");
            return;
        }
        Wire.beginTransmission(0x68); 
        Wire.write(0x1C); Wire.write(0x10); 
        if (Wire.endTransmission(true) != 0) { 
            Serial.println("MPU6050 ACCEL CFG FAILED - Check address/wiring!");
            return;
        }
        Wire.beginTransmission(0x68); 
        Wire.write(0x1B); Wire.write(0x08); 
        if (Wire.endTransmission(true) != 0) { 
            Serial.println("MPU6050 GYRO CFG FAILED - Check address/wiring!");
            return;
        }
        Serial.println("MPU6050 initialized successfully.");
    }
    void readAccel(float &ax, float &ay, float &az) {
        Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
        Wire.requestFrom(0x68, 6, true);
        int16_t x = (Wire.read()<<8)|Wire.read();
        int16_t y = (Wire.read()<<8)|Wire.read();
        int16_t z = (Wire.read()<<8)|Wire.read();
        ax = x * (8.0/32768.0) * 9.81;
        ay = y * (8.0/32768.0) * 9.81;
        az = z * (8.0/32768.0) * 9.81;
    }
    void readGyro(float &gx, float &gy, float &gz) {
        Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission(false);
        Wire.requestFrom(0x68, 6, true);
        int16_t x = (Wire.read()<<8)|Wire.read();
        int16_t y = (Wire.read()<<8)|Wire.read();
        int16_t z = (Wire.read()<<8)|Wire.read();
        gx = x / 65.5; gy = y / 65.5; gz = z / 65.5;
    }
};
MPU6050_Raw mpu;

// ---------------- QMC5883L ----------------
class QMC5883L {
public:
    void begin() {
        Serial.println("Initializing QMC5883L...");
        Wire.beginTransmission(0x0D); 
        Wire.write(0x0B); Wire.write(0x01); 
        if (Wire.endTransmission(true) != 0) { 
            Serial.println("QMC5883L RESET FAILED - Check address/wiring!");
            return;
        }
        Wire.beginTransmission(0x0D); 
        Wire.write(0x09); Wire.write(0x1D); 
        if (Wire.endTransmission(true) != 0) { 
            Serial.println("QMC5883L CONFIG FAILED - Check address/wiring!");
            return;
        }
        Serial.println("QMC5883L initialized successfully.");
    }
    bool read(int16_t &x,int16_t &y,int16_t &z) {
        Wire.beginTransmission(0x0D); Wire.write(0x00); Wire.endTransmission(false);
        Wire.requestFrom(0x0D, 6, true);
        if(Wire.available() < 6) return false;
        uint8_t x_l = Wire.read(); uint8_t x_h = Wire.read();
        uint8_t y_l = Wire.read(); uint8_t y_h = Wire.read();
        uint8_t z_l = Wire.read(); uint8_t z_h = Wire.read();

        x = (int16_t)(x_h << 8 | x_l);
        y = (int16_t)(y_h << 8 | y_l);
        z = (int16_t)(z_h << 8 | z_l);
        return true;
    }
};
QMC5883L mag;

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Wire.begin(); Wire.setClock(400000);

    // NEW: Button Setup
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor

    // OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){ Serial.println("OLED failed"); while(1);}
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.println("ML Logger Starting...");
    display.display();

    // Sensors
    mpu.begin();
    mag.begin();

    // SD Card
    if(!SD.begin(SD_CS_PIN)) { Serial.println("SD fail"); sdAvailable=false; }
    else { 
        sdAvailable=true;
        // Write headers for two files: one for fast sensor data, one for slow GPS labels
        if(!SD.exists("/sensor.csv")){
            File sensorFile = SD.open("/sensor.csv", FILE_WRITE);
            if(sensorFile){ 
                sensorFile.println("SampleID,AccX_ms2,AccY_ms2,AccZ_ms2,GyroX_dps,GyroY_dps,GyroZ_dps,MagX,MagY,MagZ"); 
                sensorFile.close();
            }
        }
        if(!SD.exists("/gps_label.csv")){
            File gpsFile = SD.open("/gps_label.csv", FILE_WRITE);
            if(gpsFile){ 
                gpsFile.println("StartSampleID,Hour,Min,Sec,Lat,Lng,Alt,Sats,Speed_kph"); 
                gpsFile.close();
            }
        }
    }
}

// ---------------- LOOP ----------------
void loop() {
    // --- Feed GPS (must run continuously) ---
    while(gpsSerial.available()>0) gps.encode(gpsSerial.read());

    // --- GPS FIX GATE CHECK ---
    // Check if we have a valid fix AND enough satellites to start logging
    if (gps.satellites.isValid() && gps.satellites.value() >= MIN_SATS_FOR_LOGGING) {
        if (!isLoggingActive) {
            isLoggingActive = true;
            Serial.println("--- GPS FIX ACHIEVED. LOGGING ACTIVATED! ---");
            display.clearDisplay(); display.setCursor(0,0);
            display.setTextSize(2); display.println("LOGGING: ON"); 
            display.setTextSize(1); display.println("GPS FIX LOCKED.");
            display.display();
        }
    } else {
        if (isLoggingActive) {
            isLoggingActive = false;
            Serial.println("--- GPS FIX LOST. LOGGING PAUSED. ---");
        }
    }

    // --- 1. GATED SAMPLING AND LOGGING ---
    if (isLoggingActive) {
        
        // --- 1.1. HIGH-FREQUENCY SENSOR SAMPLE LOOP (e.g., 50Hz) ---
        if (millis() - lastSensorSampleTime >= SENSOR_SAMPLE_MS) {
            lastSensorSampleTime = millis();

            if (sensorBufferIndex < SENSOR_BUFFER_SIZE) {
                SensorSample &s = sensorBuffer[sensorBufferIndex];
                mpu.readAccel(s.ax, s.ay, s.az);
                mpu.readGyro(s.gx, s.gy, s.gz);
                mag.read(s.mx, s.my, s.mz);
                s.sampleID = currentSampleID;
                sensorBufferIndex++; 
                currentSampleID++;
            }
        }
        
        // --- 1.2. LOW-FREQUENCY GPS SAMPLE LOOP (1Hz) ---
        if (millis() - lastGpsSampleTime >= GPS_SAMPLE_MS) {
            lastGpsSampleTime = millis();

            if (gpsBufferIndex < GPS_BUFFER_SIZE) {
                GpsSample &g = gpsBuffer[gpsBufferIndex];

                // Since logging is active, we expect these to be valid
                g.lat=gps.location.lat();
                g.lng=gps.location.lng();
                g.alt=gps.altitude.meters();
                g.sats=gps.satellites.value();
                g.speed=gps.speed.kmph();
                g.hour=gps.time.hour();
                g.minute=gps.time.minute();
                g.second=gps.time.second();
                
                g.startSampleID = currentSampleID - 1; 

                gpsBufferIndex++;
            }
        }

        // --- 1.3. AUTO-FLUSH CHECK ---
        if (sensorBufferIndex >= SENSOR_BUFFER_SIZE || gpsBufferIndex >= GPS_BUFFER_SIZE) {
            Serial.println("Buffer full. Initiating automatic flush...");
            flushBuffersToSD();
        }
    }

    // --- 2. MANUAL FLUSH/HALT CHECK (D27) --- 
    if (digitalRead(BUTTON_PIN) == LOW) {
        // Simple debouncing delay
        delay(50); 
        if (digitalRead(BUTTON_PIN) == LOW) {
            Serial.println("Manual final flush requested!");
            
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("MANUAL FLUSHING ALL DATA...");
            display.display();
            
            flushBuffersToSD();
            
            display.clearDisplay();
            display.setCursor(0, 0);
            display.setTextSize(2);
            display.println("SAVED!");
            display.setTextSize(1);
            display.println("SAFE TO POWER OFF NOW.");
            display.print("Total Flushes: "); display.println(totalFlushes);
            display.display();
            
            // Halt: Enter an infinite loop to prevent the logger from doing anything else.
            while(true) { 
                delay(1000); 
            }
        }
    }

    // --- 3. OLED Page Update (Keep outside the time gate for responsiveness) ---
    if(millis()-lastPageSwitch > PAGE_INTERVAL){
        lastPageSwitch=millis();
        oledPage=(oledPage+1)%4; // Changed from %3 to %4 to add GPS Waiting Page
        display.clearDisplay(); 
        display.setCursor(0,0);
        
        uint16_t displaySensorIndex = (sensorBufferIndex > 0) ? (sensorBufferIndex - 1) : 0;
        SensorSample &s = sensorBuffer[displaySensorIndex];
        uint8_t displayGpsIndex = (gpsBufferIndex > 0) ? (gpsBufferIndex - 1) : 0;
        GpsSample &g = gpsBuffer[displayGpsIndex];

        if (!isLoggingActive) {
            // New screen for when waiting for GPS
            display.setTextSize(2);
            display.println("WAITING...");
            display.setTextSize(1);
            display.println("Need GPS Fix > ");
            display.print(MIN_SATS_FOR_LOGGING); display.println(" SATS");
            display.print("Current Sats: "); display.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
            display.println("Keep logger stationary.");
        } else {
            // Cycle through existing status pages once logging is active
            switch(oledPage){
                case 0: // GPS Status (Lat/Lng Focus)
                case 3: // Using case 3 as well for a longer display time on key data
                    display.println("--- GPS Data (1Hz) ---");
                    display.print("Time: "); display.print(g.hour); display.print(":"); display.print(g.minute); display.print(":"); display.println(g.second);
                    display.print("Lat:"); display.println(g.lat,5);
                    display.print("Lng:"); display.println(g.lng,5);
                    display.print("Alt:"); display.print(g.alt,1); display.println("m");
                    display.print("Speed:"); display.print(g.speed,1); display.println("km/h");
                    display.print("Sats:"); display.println(g.sats);
                    break;
                    
                case 1: // Combined Sensor Readings
                    display.println("--- Sensors (50Hz) ---");
                    display.print("Sample ID: "); display.println(s.sampleID);
                    display.print("Acc X/Y/Z (m/s^2):");
                    display.print(s.ax,2); display.print("/"); display.print(s.ay,2); display.print("/"); display.println(s.az,2);
                    
                    display.print("Gyr X/Y/Z (dps):");
                    display.print(s.gx,1); display.print("/"); display.print(s.gy,1); display.print("/"); display.println(s.gz,1);
                    
                    display.print("Mag X/Y/Z (raw):");
                    display.print(s.mx); display.print("/"); display.print(s.my); display.print("/"); display.println(s.mz);
                    break;
                    
                case 2: // Buffer / SD Status
                    display.println("--- SD Status (ML Datalog) ---");
                    display.print("Total Flushes: "); display.println(totalFlushes); 
                    display.print("Sensor Buf: "); display.print(sensorBufferIndex); display.print("/"); display.println(SENSOR_BUFFER_SIZE);
                    display.print("GPS Buf: "); display.print(gpsBufferIndex); display.print("/"); display.println(GPS_BUFFER_SIZE);
                    
                    display.print("SD Status: "); display.println(sdAvailable?"OK":"FAIL");
                    display.println("Logging is fully AUTO.");
                    break;
            }
        }
        display.display();
    }
}

// ---------------- FLUSH BUFFERS ----------------
void flushBuffersToSD(){
    if(!sdAvailable) return;

    // --- 1. Write High-Frequency Sensor Data to sensor.csv ---
    if (sensorBufferIndex > 0) {
        File sensorFile = SD.open("/sensor.csv", FILE_APPEND);
        if(sensorFile){ 
            for(uint16_t i=0;i<sensorBufferIndex;i++){
                SensorSample &s = sensorBuffer[i];
                String line = String(s.sampleID)+
                              ","+String(s.ax,2)+","+String(s.ay,2)+","+String(s.az,2)+
                              ","+String(s.gx,1)+","+String(s.gy,1)+","+String(s.gz,1)+
                              ","+String(s.mx)+","+String(s.my)+","+String(s.mz);
                sensorFile.println(line);
            }
            Serial.printf("Sensor buffer flushed: %d samples written.\n", sensorBufferIndex);
            sensorFile.close();
            sensorBufferIndex = 0;
        } else {
            Serial.println("Error opening sensor.csv for appending."); sdAvailable=false;
        }
    } else {
        Serial.println("Sensor buffer empty. Skipping sensor flush.");
    }

    // --- 2. Write Low-Frequency GPS Label Data to gps_label.csv ---
    if (gpsBufferIndex > 0) {
        File gpsFile = SD.open("/gps_label.csv", FILE_APPEND);
        if(gpsFile){ 
            for(uint8_t i=0;i<gpsBufferIndex;i++){
                GpsSample &g = gpsBuffer[i];
                String line = String(g.startSampleID)+
                              ","+String(g.hour)+","+String(g.minute)+","+String(g.second)+
                              ","+String(g.lat,6)+","+String(g.lng,6)+","+String(g.alt,1)+
                              ","+String(g.sats)+","+String(g.speed,2);
                gpsFile.println(line);
            }
            Serial.printf("GPS buffer flushed: %d samples written.\n", gpsBufferIndex);
            gpsFile.close();
            gpsBufferIndex = 0;
        } else {
            Serial.println("Error opening gps_label.csv for appending."); sdAvailable=false;
        }
    } else {
        Serial.println("GPS buffer empty. Skipping GPS flush.");
    }
    
    totalFlushes++; 
    Serial.printf("Total buffer flush operation complete. Tally: %d\n", totalFlushes);
}
