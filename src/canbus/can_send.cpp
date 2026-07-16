#include <canbus/can_config.h>
#include <canbus/can_send.h>

// GPS frame index (0–15, cyclic)
static uint8_t gpsFrameIndex = 0;

bool canInit(MCP_CAN& can) {
    if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {\
        return false;
    }
    can.setMode(MCP_NORMAL);
    return true;

}

// 0x399 - Barometric pressure + temperature
void canSendBaroTemp(MCP_CAN &can, float pressurePa, float temperatureC) {
    unsigned char canMsg[8];

    // Byte 0-1: Barometric pressure in kPa * 10 (Unsigned 16-bit, LITTLE ENDIAN)
    // Convert Pa to kPa (divide by 1000), then multiply by 10 for transmission
    uint16_t pressureValue = (uint16_t)((pressurePa / 1000.0) * 10.0);
    canMsg[0] = pressureValue & 0xFF;         // LSB
    canMsg[1] = (pressureValue >> 8) & 0xFF; // MSB

    // Byte 2: Temperature in °C * 10 (Signed 8-bit)
    int8_t tempValue = (int8_t)(temperatureC * 10.0);
    canMsg[2] = (unsigned char)tempValue;

    // Bytes 3-7: Reserved (zeros)
    canMsg[3] = 0x00;
    canMsg[4] = 0x00;
    canMsg[5] = 0x00;
    canMsg[6] = 0x00;
    canMsg[7] = 0x00;

    can.sendMsgBuf(ECUMASTER_ID_0x399, 0, 8, canMsg);
}

// 0x400 - Lat / Long (zeroed)
void canSendLatLngZero(MCP_CAN& can) {
    unsigned char canMsg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can.sendMsgBuf(ECUMASTER_ID_0x400, 0, 8, canMsg);
}

void nedToLatLng(float originLat, float originLng,
                 float northM,    float eastM,
                 float& latOut,   float& lngOut)
{
    const float EARTH_R_M = 6378137.0f;
    float cosLat = cosf(originLat * DEG_TO_RAD);
    float dLat = northM / (DEG_TO_RAD * EARTH_R_M);
    float dLng = eastM  / (DEG_TO_RAD * EARTH_R_M * cosLat);
    latOut = originLat + dLat;
    lngOut = originLng + dLng;
}

// 0x400 - Lat / Long
void canSendLatLng(MCP_CAN& can, float lat, float lng) {
    // ECUMaster expects lat/lng as int32 scaled by 1e7 (degrees * 10,000,000)
    int32_t latInt = (int32_t)(lat * 1e7f);
    int32_t lngInt = (int32_t)(lng * 1e7f);

    unsigned char canMsg[8];
    // Lat in bytes 0-3, lng in bytes 4-7, big-endian
    canMsg[0] = (latInt >> 24) & 0xFF;
    canMsg[1] = (latInt >> 16) & 0xFF;
    canMsg[2] = (latInt >>  8) & 0xFF;
    canMsg[3] = (latInt >>  0) & 0xFF;
    canMsg[4] = (lngInt >> 24) & 0xFF;
    canMsg[5] = (lngInt >> 16) & 0xFF;
    canMsg[6] = (lngInt >>  8) & 0xFF;
    canMsg[7] = (lngInt >>  0) & 0xFF;

    can.sendMsgBuf(ECUMASTER_ID_0x400, 0, 8, canMsg);
}

// 0x401 - Speed, altitude, GPS info
void canSendGPSInfo(MCP_CAN& can, float altitudeMeters)
{
    unsigned char canMsg[8];
 
    // Byte 0-1: Speed (16-bit signed, km/h with scale 36/1000) - set to 0
    canMsg[0] = 0x00;
    canMsg[1] = 0x00;
 
    // Byte 2-3: Height/Altitude (16-bit signed, meters, BIG ENDIAN)
    int16_t altitudeInt = (int16_t)altitudeMeters;
    canMsg[2] = (altitudeInt >> 8) & 0xFF;  // MSB
    canMsg[3] = altitudeInt & 0xFF;          // LSB
 
    // Byte 4-5: Reserved (zeros)
    canMsg[4] = 0x00;
    canMsg[5] = 0x00;
 
    // Byte 6: GPS frame index in upper nibble (bits 4-7)
    canMsg[6] = (gpsFrameIndex << 4) & 0xF0;
 
    // Byte 7: GPS status/flags
    canMsg[7] = 0x19;
 
    can.sendMsgBuf(ECUMASTER_ID_0x401, 0, 8, canMsg);
 
    // Increment GPS frame index (0-15 cyclically)
    gpsFrameIndex = (gpsFrameIndex + 1) & 0x0F;
}

// 0x402 - Gyro X/Y
// TODO: implement send heading dynamics
void canSendGyroXY(MCP_CAN& can, float gx, float gy)
{
    unsigned char canMsg[8];

    // Byte 0-1: Heading motion (0-360) - using 0 as placeholder (BIG ENDIAN)
    uint16_t headingMotion = 0;

    // Byte 2-3: Heading vehicle (0-360) - using 0 as placeholder (BIG ENDIAN)
    uint16_t headingVehicle = 0;

    // Byte 4-5: X angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    // MPU6050 gives rad/s, convert to deg/s, then scale by 100
    // Formula: CAN_value = (rad/s * 57.2958) / 0.01 = (rad/s * 57.2958) * 100
    int16_t xAngleRate = (int16_t)(gx * 57.2958 * 100.0);

    // Byte 6-7: Y angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    int16_t yAngleRate = (int16_t)(-gy * 57.2958 * 100.0);  // Negated to correct direction


    canMsg[0] = (headingMotion >> 8) & 0xFF;  // MSB
    canMsg[1] = headingMotion & 0xFF;         // LSB

    canMsg[2] = (headingVehicle >> 8) & 0xFF;  // MSB
    canMsg[3] = headingVehicle & 0xFF;         // LSB
 
    canMsg[4] = (xAngleRate >> 8) & 0xFF;  // MSB
    canMsg[5] = xAngleRate & 0xFF;         // LSB
 
    canMsg[6] = (yAngleRate >> 8) & 0xFF;  // MSB
    canMsg[7] = yAngleRate & 0xFF;         // LSB
 
    can.sendMsgBuf(ECUMASTER_ID_0x402, 0, 8, canMsg);
}

// 0x403 - Gyro Z + accel
void canSendGyroZAccel(MCP_CAN& can,
                       float gz,
                       float ax,
                       float ay,
                       float az)
{
    unsigned char canMsg[8];

    // Byte 0-1: Z angle rate (°/s, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    int16_t zAngleRate = (int16_t)(gz * 57.2958f * 100.0f);

    // Byte 2-3: X acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    // MPU6050 gives m/s², convert to g by dividing by 9.81, then scale by 100
    // Formula: CAN_value = (m/s² / 9.81) / 0.01 = (m/s² / 9.81) * 100
    int16_t xAccel = (int16_t)((ax / 9.81f) * 100.0f);

    // Byte 4-5: Y acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    int16_t yAccel = (int16_t)((ay / 9.81f) * 100.0f);

    // Byte 6-7: Z acceleration (g, Factor: 0.01, so multiply by 100 for CAN transmission) (BIG ENDIAN)
    int16_t zAccel = (int16_t)((az / 9.81f) * 100.0f);



    canMsg[0] = (zAngleRate >> 8) & 0xFF;  // MSB
    canMsg[1] = zAngleRate & 0xFF;         // LSB

    canMsg[2] = (xAccel >> 8) & 0xFF;  // MSB
    canMsg[3] = xAccel & 0xFF;         // LSB

    canMsg[4] = (yAccel >> 8) & 0xFF;  // MSB
    canMsg[5] = yAccel & 0xFF;         // LSB

    canMsg[6] = (zAccel >> 8) & 0xFF;  // MSB
    canMsg[7] = zAccel & 0xFF;         // LSB
 
    can.sendMsgBuf(ECUMASTER_ID_0x403, 0, 8, canMsg);
}

// 0x404 - UTC time (stub)
// TODO: feed in UTC time from gps and/or RTC
void canSendUTCTime(MCP_CAN& can)
{
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
 
    can.sendMsgBuf(ECUMASTER_ID_0x404, 0, 8, canMsg);
}
