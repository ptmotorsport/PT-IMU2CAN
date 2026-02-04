
#pragma once
#include <Arduino.h>
#include <mcp_can.h>

// Initialize CAN 
bool canInit(MCP_CAN& can);

// ---- ECUMaster transmit functions ----

// 0x399: Barometric pressure + temperature
void canSendBaroTemp(MCP_CAN& can, float pressurePa, float temperatureC);

// 0x400: Latitude / Longitude (currently zeroed)
void canSendLatLngZero(MCP_CAN& can);

// 0x401: Speed, altitude, GPS info
void canSendGPSInfo(MCP_CAN& can, float altitudeMeters);

// 0x402: Gyro X/Y
void canSendGyroXY(MCP_CAN& can, float gyroX_rad_s, float gyroY_rad_s);

// 0x403: Gyro Z + accelerations
void canSendGyroZAccel(MCP_CAN& can,
                       float gyroZ_rad_s,
                       float accelX_ms2,
                       float accelY_ms2,
                       float accelZ_ms2);

// 0x404: UTC time (stub for now)
void canSendUTCTime(MCP_CAN& can);
