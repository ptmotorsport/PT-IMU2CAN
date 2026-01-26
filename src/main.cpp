#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <mcp_can.h>

// ECUMaster CAN IDs
#define ECUMASTER_ID_0x399  0x399  // BMP280 Barometric pressure and temperature
#define ECUMASTER_ID_0x400  0x400  // Latitude, Longitude
#define ECUMASTER_ID_0x401  0x401  // Speed, Height, Satellites, GPS frame index, GPS status
#define ECUMASTER_ID_0x402  0x402  // Heading motion, Heading vehicle, X angle rate, Y angle rate
#define ECUMASTER_ID_0x403  0x403  // Z angle rate, X/Y/Z acceleration
#define ECUMASTER_ID_0x404  0x404  // UTC time

// CAN CS pin
#define CAN_CS_PIN 10

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
MCP_CAN CAN(CAN_CS_PIN);  // CS pin 10 for SPI

// Timing variables
unsigned long lastCANSend = 0;
unsigned long startupTime = 0;
const unsigned long STARTUP_DURATION = 2000;  // 2 seconds startup sequence
const unsigned long STARTUP_INTERVAL = 100;   // 10Hz during startup (100ms)
const unsigned long NORMAL_INTERVAL = 40;     // 25Hz normal operation (40ms)
bool startupComplete = false;

// GPS frame index counter
uint8_t gpsFrameIndex = 0;

// Function to convert float to 16-bit signed integer with scaling
int16_t floatToInt16(float value, float scale) {
  return (int16_t)(value * scale);
}

// Function to convert float to 16-bit unsigned integer
uint16_t floatToUInt16(float value) {
  return (uint16_t)value;
}

// Function to send ECUMaster ID 0x400 (Latitude, Longitude) - all zeros
void sendCANMessage_0x400() {
  unsigned char canMsg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(ECUMASTER_ID_0x400, 0, 8, canMsg);
}

// Function to send CAN ID 0x399 (BMP280 Barometric pressure and temperature)
void sendCANMessage_0x399(float pressure, float temperature) {
  unsigned char canMsg[8];
  
  // Byte 0-1: Barometric pressure in kPa * 10 (Unsigned 16-bit, LITTLE ENDIAN)
  // Convert Pa to kPa (divide by 1000), then multiply by 10 for transmission
  uint16_t pressureValue = (uint16_t)((pressure / 1000.0) * 10.0);
  canMsg[0] = pressureValue & 0xFF;         // LSB
  canMsg[1] = (pressureValue >> 8) & 0xFF; // MSB
  
  // Byte 2: Temperature in °C * 10 (Signed 8-bit)
  int8_t tempValue = (int8_t)(temperature * 10.0);
  canMsg[2] = (unsigned char)tempValue;
  
  // Bytes 3-7: Reserved (zeros)
  canMsg[3] = 0x00;
  canMsg[4] = 0x00;
  canMsg[5] = 0x00;
  canMsg[6] = 0x00;
  canMsg[7] = 0x00;
  
  CAN.sendMsgBuf(ECUMASTER_ID_0x399, 0, 8, canMsg);
}

// Function to send ECUMaster ID 0x401 (Speed, Height, Satellites, GPS info)
void sendCANMessage_0x401(float altitude) {
  unsigned char canMsg[8];
  
  // Byte 0-1: Speed (16-bit signed, km/h with scale 36/1000) - set to 0
  canMsg[0] = 0x00;
  canMsg[1] = 0x00;
  
  // Byte 2-3: Height/Altitude (16-bit signed, meters, BIG ENDIAN)
  int16_t altitudeInt = (int16_t)altitude;
  canMsg[2] = (altitudeInt >> 8) & 0xFF;  // MSB
  canMsg[3] = altitudeInt & 0xFF;          // LSB
  
  // Byte 4-5: Reserved (zeros)
  canMsg[4] = 0x00;
  canMsg[5] = 0x00;
  
  // Byte 6: GPS frame index in upper nibble (bits 4-7)
  canMsg[6] = (gpsFrameIndex << 4) & 0xF0;
  
  // Byte 7: GPS status/flags
  canMsg[7] = 0x19;
  
  CAN.sendMsgBuf(ECUMASTER_ID_0x401, 0, 8, canMsg);
  
  // Increment GPS frame index (0-15 cyclically)
  gpsFrameIndex = (gpsFrameIndex + 1) & 0x0F;
}

// Function to send ECUMaster ID 0x404 (UTC time)
void sendCANMessage_0x404() {
  unsigned char canMsg[8];
  
  // Send a fixed date/time to match genuine device pattern
  // In real implementation, you would get time from GPS or RTC module
  canMsg[0] = 0x14;   // UTC year offset from 2000 (0x14 = 20 = year 2020)
  canMsg[1] = 0x08;   // UTC month (8 = August)
  canMsg[2] = 0x02;   // UTC day (2)
  canMsg[3] = 0x00;   // UTC hour (0-23)
  canMsg[4] = 0x00;   // UTC minute (0-59)
  canMsg[5] = 0x00;   // UTC second (0-60)
  canMsg[6] = 0x00;   // UTC millisecond LSB
  canMsg[7] = 0x00;   // UTC millisecond MSB
  
  CAN.sendMsgBuf(ECUMASTER_ID_0x404, 0, 8, canMsg);
}

// Function to send ECUMaster ID 0x402 (gyro rates)
void sendCANMessage_0x402(float gyroX, float gyroY) {
  unsigned char canMsg[8];
  
  // Byte 0-1: Heading motion (0-360) - using 0 as placeholder (BIG ENDIAN)
  uint16_t headingMotion = 0;
  canMsg[0] = (headingMotion >> 8) & 0xFF;  // MSB
  canMsg[1] = headingMotion & 0xFF;         // LSB
  
  // Byte 2-3: Heading vehicle (0-360) - using 0 as placeholder (BIG ENDIAN)
  uint16_t headingVehicle = 0;
  canMsg[2] = (headingVehicle >> 8) & 0xFF;  // MSB
  canMsg[3] = headingVehicle & 0xFF;         // LSB
  
  // Byte 4-5: X angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  // MPU6050 gives rad/s, convert to deg/s, then scale by 100
  // Formula: CAN_value = (rad/s * 57.2958) / 0.01 = (rad/s * 57.2958) * 100
  int16_t xAngleRate = (int16_t)(gyroX * 57.2958 * 100.0);
  canMsg[4] = (xAngleRate >> 8) & 0xFF;  // MSB
  canMsg[5] = xAngleRate & 0xFF;         // LSB
  
  // Byte 6-7: Y angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  int16_t yAngleRate = (int16_t)(-gyroY * 57.2958 * 100.0);  // Negated to correct direction
  canMsg[6] = (yAngleRate >> 8) & 0xFF;  // MSB
  canMsg[7] = yAngleRate & 0xFF;         // LSB
  
  CAN.sendMsgBuf(ECUMASTER_ID_0x402, 0, 8, canMsg);
}

// Function to send ECUMaster ID 0x403 (Z gyro and accelerations)
void sendCANMessage_0x403(float gyroZ, float accelX, float accelY, float accelZ) {
  unsigned char canMsg[8];
  
  // Byte 0-1: Z angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  int16_t zAngleRate = (int16_t)(gyroZ * 57.2958 * 100.0);
  canMsg[0] = (zAngleRate >> 8) & 0xFF;  // MSB
  canMsg[1] = zAngleRate & 0xFF;         // LSB
  
  // Byte 2-3: X acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  // MPU6050 gives m/s², convert to g by dividing by 9.81, then scale by 100
  // Formula: CAN_value = (m/s² / 9.81) / 0.01 = (m/s² / 9.81) * 100
  int16_t xAccel = (int16_t)((accelX / 9.81) * 100.0);
  canMsg[2] = (xAccel >> 8) & 0xFF;  // MSB
  canMsg[3] = xAccel & 0xFF;         // LSB
  
  // Byte 4-5: Y acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  int16_t yAccel = (int16_t)((accelY / 9.81) * 100.0);
  canMsg[4] = (yAccel >> 8) & 0xFF;  // MSB
  canMsg[5] = yAccel & 0xFF;         // LSB
  
  // Byte 6-7: Z acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
  int16_t zAccel = (int16_t)((accelZ / 9.81) * 100.0);
  canMsg[6] = (zAccel >> 8) & 0xFF;  // MSB
  canMsg[7] = zAccel & 0xFF;         // LSB
  
  CAN.sendMsgBuf(ECUMASTER_ID_0x403, 0, 8, canMsg);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("ECUMaster IMU/Baro to CAN Test");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize BMP280
  Serial.println("Initializing BMP280...");
  if (!bmp.begin(0x76)) {  // Try address 0x76 first
    if (!bmp.begin(0x77)) {  // Try address 0x77
      Serial.println("Failed to find BMP280 chip!");
      while (1) delay(10);
    }
  }
  Serial.println("BMP280 Found!");
  
  // Configure BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
  
  // Initialize MCP2515
  Serial.println("Initializing MCP2515...");
  
  // Initialize CAN at 1Mbps with 8MHz crystal
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1) delay(10);
  }
  
  // Set to normal mode
  CAN.setMode(MCP_NORMAL);
  
  Serial.println("Setup complete! Starting startup sequence...");
  startupTime = millis();
  delay(100);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if startup sequence is complete
  if (!startupComplete && (currentMillis - startupTime >= STARTUP_DURATION)) {
    startupComplete = true;
    Serial.println("Startup complete! Switching to 25Hz operation...");
  }
  
  // Determine current send interval based on startup state
  unsigned long currentInterval = startupComplete ? NORMAL_INTERVAL : STARTUP_INTERVAL;
  
  // Send CAN messages at specified interval
  if (currentMillis - lastCANSend >= currentInterval) {
    lastCANSend = currentMillis;
    
    // Read MPU6050 sensor data
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Read BMP280 sensor data
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1018.0); // Use local sea level pressure in hPa (adjust as needed)
    
    // Send ECUMaster CAN messages in correct order (matching genuine device)
    sendCANMessage_0x399(pressure, temperature);  // BMP280 Baro and Temp
    sendCANMessage_0x400();  // Lat/Long (all zeros)
    sendCANMessage_0x404();  // UTC time
    sendCANMessage_0x401(altitude);  // Speed, Height, GPS info
    sendCANMessage_0x402(gyro.gyro.x, gyro.gyro.y);
    sendCANMessage_0x403(gyro.gyro.z, accel.acceleration.x, 
                         accel.acceleration.y, accel.acceleration.z);
    
    // Print data to Serial for debugging (comment out if not needed)
    Serial.print("Gyro X: "); Serial.print(gyro.gyro.x * 57.2958); Serial.print(" °/s, ");
    Serial.print("Y: "); Serial.print(gyro.gyro.y * 57.2958); Serial.print(" °/s, ");
    Serial.print("Z: "); Serial.print(gyro.gyro.z * 57.2958); Serial.println(" °/s");
    
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x / 9.81); Serial.print(" g, ");
    Serial.print("Y: "); Serial.print(accel.acceleration.y / 9.81); Serial.print(" g, ");
    Serial.print("Z: "); Serial.print(accel.acceleration.z / 9.81); Serial.println(" g");
    
    Serial.print("Temp: "); Serial.print(temperature); Serial.print(" °C, ");
    Serial.print("Pressure: "); Serial.print(pressure / 100.0); Serial.print(" hPa, ");
    Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
    Serial.println();
  }
}
