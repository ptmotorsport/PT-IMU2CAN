#include "gps.h"

// ---- GPS parser ----
static TinyGPSPlus gps;

#define GPS_SERIAL Serial1   // hardware UART

GPSSample readGPS()
{
    GPSSample sample;
    sample.valid = false;

    while (GPS_SERIAL.available()) {
        gps.encode(GPS_SERIAL.read());
    }

    if (gps.location.isValid()) {
        sample.valid = true;

        // NOTE: lat/lng still in degrees at this point
        // Might need to convert to local ENU/NED later for EKF
        sample.position(0) = gps.location.lat();
        sample.position(1) = gps.location.lng();
        sample.position(2) = gps.altitude.meters();

        sample.speed_mps = gps.speed.mps();
        sample.satellites = gps.satellites.value();
    }

    return sample;
}
