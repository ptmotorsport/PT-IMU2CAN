
#include "api/Common.h"
#include "mcp_can.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <canbus/can_send.h>
#include <imu.h>
#include <gps.h>
#include <ekf.h>

#define DEBUG_SERIAL 1   // 1 = enabled, 0 = disabled

#if DEBUG_SERIAL
  #define DBG_BEGIN(baud) Serial.begin(baud)
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)
#else
  #define DBG_BEGIN(baud)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

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
    DBG_BEGIN(115200);
    while (DEBUG_SERIAL && !Serial) delay(10);

    DBG_PRINTLN("Beginning initialization process");


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
    static unsigned long lastLoopTime = 0;
    static unsigned long lastEKFTime  = 0;

    unsigned long now = millis();

    // ---- Startup mode switch ----
    if (!startupComplete && (now - startupTime >= STARTUP_DURATION)) {
        startupComplete = true;
        Serial.println("Startup complete! Switching to 25Hz operation...");
    }

    unsigned long interval = startupComplete ? NORMAL_INTERVAL : STARTUP_INTERVAL;

    // ---- Rate gate ----
    if (now - lastEKFTime < interval) {
        return; // too soon, skip this loop iteration
    }

    // ---- dt computation (ONLY when EKF runs) ----
    float dt;
    if (lastEKFTime == 0) {
        dt = interval * 0.001f;
    } else {
        dt = (now - lastEKFTime) * 0.001f;
    }
    lastEKFTime = now;

    // Clamp dt
    if (dt <= 0.0f || dt > 0.1f) {
        dt = interval * 0.001f;
    }

    // ---- Sensor reads ----
    GPSSample gps_s = readGPS();
    IMUSample imu1  = readIMU(IMU_1);
    IMUSample imu2  = readIMU(IMU_2);

    // To get R: hold the car perfectly still, print both IMUs' raw accel readings,
    // find the rotation that maps IMU2's gravity vector onto IMU1's. 
    // For the arm, just measure with a ruler in metres from each IMU to the car's centre of mass.

    // IMU1 is body-frame reference — identity rotation, measure arm from CoM
    static const IMUAlignment align1 = { Identity<3>(), {0.0f, 0.0f, 0.0f} };

    // IMU2 — set R to match physical mounting, measure arm with a ruler
    static const IMUAlignment align2 = { Identity<3>(), {0.0f, 0.0f, 0.0f} };

    IMUSample imu1_body = applyAlignment(imu1, align1, imu1.gyro);
    IMUSample imu2_body = applyAlignment(imu2, align2, imu1_body.gyro);
    IMUSample imu_fused = fuseIMUs(imu1_body, imu2_body);


    // ---- EKF PREDICT ----
    ekf.predict(imu_fused.accel, imu_fused.gyro, dt);

    // ---- EKF UPDATE ----
    if (gps_s.valid && gpsHasFix()) {
        ekf.updateGPS(gps_s.position);
    }

    const StateVector& INS_State = ekf.getState(); // take snapshot

    // ---- DEBUG (post-EKF, throttled) ----
#if DEBUG_SERIAL
    static unsigned long lastDebugPrint = 0;
    if (now - lastDebugPrint >= 100) { // 10 Hz
        lastDebugPrint = now;

        DBG_PRINT("t=");
        DBG_PRINT(now);

        DBG_PRINT(" | GPS=");
        DBG_PRINT(gps_s.valid ? "OK" : "NO");

        DBG_PRINT(" Fix=");
        DBG_PRINT(gpsHasFix());

        DBG_PRINT(" | Lat=");
        DBG_PRINT(gps_s.position(0));
        DBG_PRINT(" Lng=");
        DBG_PRINT(gps_s.position(1));
        DBG_PRINT(" Alt=");
        DBG_PRINT(gps_s.position(2));
        DBG_PRINT(" Spd=");
        DBG_PRINT(gps_s.speed_mps);

        DBG_PRINT(" | IMU1 a=(");
        DBG_PRINT(imu1.accel(0)); DBG_PRINT(",");
        DBG_PRINT(imu1.accel(1)); DBG_PRINT(",");
        DBG_PRINT(imu1.accel(2)); DBG_PRINT(")");

        DBG_PRINT(" | IMU2 a=(");
        DBG_PRINT(imu2.accel(0)); DBG_PRINT(",");
        DBG_PRINT(imu2.accel(1)); DBG_PRINT(",");
        DBG_PRINTLN(imu2.accel(2));

        DBG_PRINTLN("INS STATE =(");
        DBG_PRINT(INS_State); DBG_PRINT(")");
    }
#endif

    /* TODO: Implement State loging 
     * - Take/overwrite INS snapshot of current vehicle state   
     * - Check time since last CAN send
     * - Send to ECU MASTER over CAN via defined methods in prefered order
     */


    // 0x400: lat/lng from EKF position states
    canSendLatLng(CAN, INS_State(EKF::PX), INS_State(EKF::PY));

    // 0x401: altitude + GPS info
    canSendGPSInfo(CAN, INS_State(EKF::PZ));

    // 0x402: gyro X/Y
    canSendGyroXY(CAN, INS_State(EKF::ROLL), INS_State(EKF::PITCH));  // or raw gyro if preferred

    // 0x403: gyro Z + accelerations (use raw IMU values, not EKF states)
    canSendGyroZAccel(CAN,
        imu1.gyro(2)  - INS_State(EKF::BGZ),
        imu1.accel(0) - INS_State(EKF::BAX),
        imu1.accel(1) - INS_State(EKF::BAY),
        imu1.accel(2) - INS_State(EKF::BAZ));

    // 0x399: baro (if you have a sensor)
    // canSendBaroTemp(CAN, pressurePa, temperatureC);

    // 0x404: UTC stub
    canSendUTCTime(CAN);
}
