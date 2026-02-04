#include "api/Common.h"
#include <cstdint>
#include <gps.h>
#include <Arduino.h>
#include <iterator>

#define GPS_SERIAL          Serial1   // hardware UART
#define GPS_BAUD            9600
#define GPS_INIT_TIMEOUT    1500

// ---- GPS parser ----
static TinyGPSPlus gps;
static uint32_t lastFixMillis = 0;


// Initialize GPS
// What it does: ensures uart has consistent nmea stream before it times out, Meaning the gps is present and consistently sendind data.
// TODO: Create extension method that finds the 'first lock' of the gps
bool initGPS()
{
    GPS_SERIAL.begin(GPS_BAUD);

    uint32_t start = millis();
    bool dataDetected = false;

    // Drain + detect incoming nmea 
    while (millis() - start < GPS_INIT_TIMEOUT) {
        while (GPS_SERIAL.available()) {
            char c = GPS_SERIAL.read();
            gps.encode(c);
            dataDetected = true;
        }
    }
    return dataDetected;
}

bool gpsHasFix()
{
    // Must have had a valid fix at least once
    if (!gps.location.isValid()) {
        return false;
    }

    // Require minimum satellites
    if (gps.satellites.value() < 3) {
        return false;
    }

    // Optional: freshness timeout (recommended)
    const uint32_t FIX_TIMEOUT_MS = 2000;
    if (millis() - lastFixMillis > FIX_TIMEOUT_MS) {
        return false;
    }

    return true;
}

// Function to get GPS SAMPLE
// What it does: Creates a Sample object and then fills it in via tiny gps API calls
GPSSample readGPS()
{
    GPSSample sample;
    sample.valid = false;

    while (GPS_SERIAL.available()) {
        gps.encode(GPS_SERIAL.read());
    }

    if (gps.location.isUpdated()){
        sample.valid = true;

        // NOTE: lat/lng still in degrees at this point
        // Might need to convert to local ENU/NED later for EKF
        sample.position(0) = gps.location.lat();
        sample.position(1) = gps.location.lng();
        sample.position(2) = gps.altitude.meters();

        sample.speed_mps = gps.speed.mps();
        sample.satellites = gps.satellites.value();

        // mark time of last valid fix
        if (gps.location.isValid()) {
            lastFixMillis = millis();
        }
    }

    return sample;
}


