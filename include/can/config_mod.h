/* 
 * Main Config file for CanBus addresses, constants, scalers
*/
#pragma once
#include <stdint.h>

// ---- ECUMaster CAN IDs ----
constexpr uint16_t ECUMASTER_ID_0x399 = 0x399; // Baro + Temp
constexpr uint16_t ECUMASTER_ID_0x400 = 0x400; // Lat / Lon
constexpr uint16_t ECUMASTER_ID_0x401 = 0x401; // Speed, Height, GPS info
constexpr uint16_t ECUMASTER_ID_0x402 = 0x402; // Gyro X/Y
constexpr uint16_t ECUMASTER_ID_0x403 = 0x403; // Gyro Z + Accels
constexpr uint16_t ECUMASTER_ID_0x404 = 0x404; // UTC time

// ---- Scaling constants ----
constexpr float RAD_TO_DEG = 57.2957795f;
constexpr float MS2_TO_G   = 1.0f / 9.81f;

// ---- GPS ---- 
constexpr uint8_t GPS_FRAME_MAX = 16;
