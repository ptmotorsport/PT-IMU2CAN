#include <Arduino.h>
#include <Wire.h>
#include <imu.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



// ---- One instance per possible address ----
static Adafruit_MPU6050 mpu_68;
static Adafruit_MPU6050 mpu_69;

static bool imu68_init = false;
static bool imu69_init = false;

// Helper: get the correct MPU object, ensure no random sensor.
static Adafruit_MPU6050& getMPU(uint8_t address) {
    return (address == 0x69) ? mpu_69 : mpu_68;
}

// Helper: get init flag
static bool& getInitFlag(uint8_t address) {
    return (address == 0x69) ? imu69_init : imu68_init;
}

bool initIMU(uint8_t address)
{
    Adafruit_MPU6050& mpu = getMPU(address);
    bool& initialized = getInitFlag(address);

    if (initialized) {
        return true;  // already initialized
    }

    if (!mpu.begin(address)) {
        return false;
    }

    // ---- Configure IMU ONCE ----
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    initialized = true;
    return true;
}

IMUSample readIMU(uint8_t address)
{
    IMUSample sample; // Init matrices

    Adafruit_MPU6050& mpu = getMPU(address);

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Acceleration (m/s^2)
    sample.accel(0) = accel.acceleration.x;
    sample.accel(1) = accel.acceleration.y;
    sample.accel(2) = accel.acceleration.z;

    // Angular rate (rad/s)
    sample.gyro(0) = gyro.gyro.x;
    sample.gyro(1) = gyro.gyro.y;
    sample.gyro(2) = gyro.gyro.z;

    sample.timestamp = millis() * 0.001f;
    return sample;
}

