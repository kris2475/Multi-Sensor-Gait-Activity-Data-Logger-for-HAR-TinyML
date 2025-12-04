/*
 * ESP32 Multi-Sensor Logger & Display
 * * * Hardware:
 * - ESP32 Dev Module
 * - GPS: Neo-6M (Serial2) -> RX:16, TX:17
 * - OLED: SSD1306 128x64 (I2C 0x3C) -> SDA:21, SCL:22
 * - Accel/Gyro: MPU6050 (I2C 0x68) -> SDA:21, SCL:22
 * - Compass: QMC5883L (I2C 0x0D) -> SDA:21, SCL:22
 * - Micro SD Card (VSPI) -> CS:5, SCK:18, MISO:19, MOSI:23
 * * * Dependencies (Install via Library Manager):
 * - Adafruit SSD1306
 * - TinyGPSPlus
 * - SD (Standard Arduino Library)
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// --- Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// SD Card SPI Pins (standard VSPI)
#define SD_CS_PIN 5

// GPS UART Pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

// --- Objects ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use UART2

// --- Global Variables for Logging ---
unsigned long lastLogTime = 0;
unsigned long lastDisplayUpdate = 0;
bool sdAvailable = false;
File logFile;

// --- MPU6050 (0x68) Direct Driver ---
// Bypasses library ID checks that fail on clones
class MPU6050_Raw {
  public:
    // MPU6050 registers and configuration
    void begin() {
      // 1. Wake up MPU6050 (Write 0 to PWR_MGMT_1)
      Wire.beginTransmission(0x68);
      Wire.write(0x6B); // PWR_MGMT_1 register
      Wire.write(0x00); // Wake up
      Wire.endTransmission();
      
      delay(10); 

      // 2. Set Accelerometer Range to +/- 8G (Sensitivity 4096 LSB/g)
      Wire.beginTransmission(0x68);
      Wire.write(0x1C); // ACCEL_CONFIG register
      Wire.write(0x10); // 0x10 = 8G range
      Wire.endTransmission();

      // 3. Set Gyroscope Range to +/- 500 deg/s (Sensitivity 65.5 LSB/(deg/s))
      Wire.beginTransmission(0x68);
      Wire.write(0x1B); // GYRO_CONFIG register
      Wire.write(0x08); // 0x08 = 500 deg/s range
      Wire.endTransmission();
    }

    void readAccel(float &ax, float &ay, float &az) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); // Start reading at ACCEL_XOUT_H
      Wire.endTransmission();

      Wire.requestFrom(0x68, 6);
      if (Wire.available() < 6) return;

      int16_t rawX = (Wire.read() << 8) | Wire.read();
      int16_t rawY = (Wire.read() << 8) | Wire.read();
      int16_t rawZ = (Wire.read() << 8) | Wire.read();

      // Convert raw reading to m/s^2 (4096 LSB/g for 8G range)
      ax = (rawX / 4096.0) * 9.81;
      ay = (rawY / 4096.0) * 9.81;
      az = (rawZ / 4096.0) * 9.81;
    }

    void readGyro(float &gx, float &gy, float &gz) {
      Wire.beginTransmission(0x68);
      Wire.write(0x43); // Start reading at GYRO_XOUT_H
      Wire.endTransmission();

      Wire.requestFrom(0x68, 6);
      if (Wire.available() < 6) return;

      int16_t rawX = (Wire.read() << 8) | Wire.read();
      int16_t rawY = (Wire.read() << 8) | Wire.read();
      int16_t rawZ = (Wire.read() << 8) | Wire.read();

      // Convert raw reading to deg/s (65.5 LSB/(deg/s) for 500 deg/s range)
      gx = rawX / 65.5;
      gy = rawY / 65.5;
      gz = rawZ / 65.5;
    }
};

MPU6050_Raw mpu;

// --- QMC5883L (0x0D) Direct Driver ---
class QMC5883L {
  public:
    void begin() {
      Wire.beginTransmission(0x0D);
      Wire.write(0x0B); // Set/Reset Period Register
      Wire.write(0x01);
      Wire.endTransmission();

      Wire.beginTransmission(0x0D);
      Wire.write(0x09); // Control Register 1
      Wire.write(0x1D); // Mode: Continuous, ODR: 200Hz, RNG: 8G, OSR: 512
      Wire.endTransmission();
    }

    bool read(int16_t &x, int16_t &y, int16_t &z) {
      Wire.beginTransmission(0x0D);
      Wire.write(0x00); // Start reading from X LSB
      Wire.endTransmission();
      
      Wire.requestFrom(0x0D, 6);
      if (Wire.available() < 6) return false;
      
      x = (int16_t)(Wire.read() | Wire.read() << 8);
      y = (int16_t)(Wire.read() | Wire.read() << 8);
      z = (int16_t)(Wire.read() | Wire.read() << 8);
      return true;
    }
};

QMC5883L mag;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // 1. Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // fast mode

  // 2. Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  // 3. Initialize MPU6050 (Direct)
  mpu.begin();
  Serial.println("MPU6050 Init sent");

  // 4. Initialize QMC5883L (Direct)
  mag.begin();
  Serial.println("QMC5883L Init sent");
  
  // 5. Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Card failed, or not present.");
    sdAvailable = false;
  } else {
    Serial.println("SD card initialized.");
    sdAvailable = true;
    
    // Write CSV Header if file doesn't exist
    if (!SD.exists("/datalog.csv")) {
      logFile = SD.open("/datalog.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("Time,Lat,Lon,Alt,Sats,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
        logFile.close();
        Serial.println("Wrote CSV header.");
      } else {
        Serial.println("Error opening datalog.csv to write header!");
      }
    }
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Sensors Ready. SD: ");
  display.println(sdAvailable ? "OK" : "FAIL");
  display.display();
  delay(1000);
}

void loop() {
  // Feed GPS parser constantly
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // --- Read Sensors ---
  float ax, ay, az;
  float gx, gy, gz;
  int16_t magX, magY, magZ;
  
  mpu.readAccel(ax, ay, az);
  mpu.readGyro(gx, gy, gz);
  mag.read(magX, magY, magZ);

  // Task 1: Update Display (smoother viewing, e.g., 4 times per second)
  if (millis() - lastDisplayUpdate >= 250) {
    lastDisplayUpdate = millis();

    display.clearDisplay();
    display.setCursor(0,0);
    
    // Line 0: GPS Status
    display.print("GPS: ");
    if(gps.location.isValid()) {
        display.print(gps.location.lat(), 4); 
        display.print("/"); 
        display.println(gps.location.lng(), 4);
    } else {
        display.print("No Fix (Sats:"); 
        display.print(gps.satellites.value());
        display.println(")");
    }
    display.println("---------------------");

    // Line 2: Accel Z + SD Status
    display.print("A Z: "); display.print(az, 2); 
    display.print(" G Z: "); display.print(gz, 1);
    display.print(" M Z:"); display.println(magZ);
    
    // Line 3: Accel X/Y
    display.print("A X/Y: "); display.print(ax, 2); display.print(" / "); display.println(ay, 2);

    // Line 4: Gyro X/Y
    display.print("G X/Y: "); display.print(gx, 1); display.print(" / "); display.println(gy, 1);
    
    // Line 5: Mag X/Y
    display.print("M X/Y: "); display.print(magX); display.print(" / "); display.println(magY);
    
    // Line 6: Logging Status
    display.setCursor(0, 56); // Bottom line
    display.print("Log Status: ");
    display.print(sdAvailable ? "SD OK" : "SD FAIL");

    display.display();
  }


  // Task 2: Log to SD Card (once per second, as requested)
  if (millis() - lastLogTime >= 1000) {
    lastLogTime = millis();

    // Generate CSV data string
    String dataString = "";
    
    // Time (uses GPS time if valid, otherwise internal millis)
    if (gps.time.isValid()) {
      char timeBuff[10];
      sprintf(timeBuff, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      dataString += String(timeBuff);
    } else {
      dataString += String(millis()/1000.0, 3);
    }
    
    dataString += "," + String(gps.location.lat(), 6);
    dataString += "," + String(gps.location.lng(), 6);
    dataString += "," + String(gps.altitude.meters(), 1);
    dataString += "," + String(gps.satellites.value());
    
    dataString += "," + String(ax, 3);
    dataString += "," + String(ay, 3);
    dataString += "," + String(az, 3);

    dataString += "," + String(gx, 2);
    dataString += "," + String(gy, 2);
    dataString += "," + String(gz, 2);
    
    dataString += "," + String(magX);
    dataString += "," + String(magY);
    dataString += "," + String(magZ);

    // Write to SD
    if (sdAvailable) {
      logFile = SD.open("/datalog.csv", FILE_APPEND);
      if (logFile) {
        logFile.println(dataString);
        logFile.close();
        // Log to Serial as confirmation
        Serial.println("SD Log: " + dataString);
      } else {
        Serial.println("Error opening datalog.csv for logging!");
        // Set flag to false if write fails
        sdAvailable = false; 
      }
    } else {
      // Print to Serial if SD is not available
      Serial.println("No SD Card: " + dataString); 
    }
  }
}


