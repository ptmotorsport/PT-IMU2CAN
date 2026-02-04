
#include "api/Common.h"
#include "mcp_can.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <canbus/can_send.h>
#include <imu.h>
#include <gps.h>
#include <ekf.h>

#define GPS_BAUD 9600
#define CAN_CS_PIN 10

#define IMU_1 0x68
#define IMU_2 0x69

MCP_CAN CAN(CAN_CS_PIN);

// Timing variables
unsigned long lastCANSend = 0;
unsigned long startupTime = 0;
const unsigned long STARTUP_DURATION = 2000;  // 2 seconds startup sequence
const unsigned long STARTUP_INTERVAL = 100;   // 10Hz during startup (100ms)
const unsigned long NORMAL_INTERVAL = 40;     // 25Hz normal operation (40ms)
bool startupComplete = false;

// Extended Kalman filter instance 
EKF ekf;


void setup() 
{
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("Begining initialization process");

    Wire.begin(); // i2c begin
  
    Serial.println("Initializing MPU_1"); // mpu6050
    if (!initIMU(IMU_1)) {
      Serial.println("Failed to find MPU_1");
      while (1) delay(10);
    }
    Serial.println("Found MPU_1");
  
    Serial.println("Initializing MPU_2");
    if (!initIMU(IMU_2)) {
      Serial.println("Failed to find MPU_2");
      while (1) delay(10);
    }
    Serial.println("Found MPU_2, Success");

    // ignore barometter for now.
  
    // start GPS 
    Serial.println("Initializing GPS"); // ublox neo 7m
    if (!initGPS()) {
      Serial.println("Failed to find GPS (no UART data)");
      while (1) delay(10);
    }
    Serial.println("GPS UART active, Success");

    Serial.println("Initializing MCP2515");
    if (!canInit(CAN)) {
      Serial.println("Failed to initialize MCP2515");
      while (1) delay(10);
    }
    Serial.println("MCP2515 Initialized Succesfully");

    // zeroing ekf, placeholder
    // TODO: use GPS lock to zero state 
    StateVector x0;
    x0.Fill(0.0f);      // start at origin, zero velocity, zero attitude
    ekf.init(x0);
    
    Serial.println("Setup complete! Starting startup sequence...");
    startupTime = millis();
    delay(100);
}

void loop() 
{
    unsigned long currentMillis = millis();
    
    // Check if startup sequence is complete
    if (!startupComplete && (currentMillis - startupTime >= STARTUP_DURATION)) {
      startupComplete = true;
      Serial.println("Startup complete! Switching to 25Hz operation...");
    }
    
    // Determine current send interval based on startup state
    unsigned long currentInterval = startupComplete ? NORMAL_INTERVAL : STARTUP_INTERVAL;

    GPSSample gps_s = readGPS();

    IMUSample imu1  = readIMU(IMU_1);
    IMUSample imu2  = readIMU(IMU_2);

    /* TODO: Rigid Body Frame alignment on imus, then apply some sort fo filter, 
     * start with average etc.
     * just use imu1 for testing ekf, keep imu2 for mem and validity testing. 
     */

    if (gps_s.valid && gpsHasFix()) {
        ekf.updateGPS(gps_s.position);
    }

    /* TODO: Implement State loging 
     * - Take/overwrite INS snapshot of current vehicle state
     * - Check time since last CAN send
     * - Send to ECU MASTER over CAN via defined methods in prefered order
     */
}
