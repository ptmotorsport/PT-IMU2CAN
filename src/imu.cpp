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

// Helper: get the correct MPU object
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

    if (initialized) return true;

    if (!mpu.begin(address)) return false;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    initialized = true;
    return true;
}

IMUSample readIMU(uint8_t address)
{
    IMUSample sample;
    Adafruit_MPU6050& mpu = getMPU(address);
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    sample.accel(0) = accel.acceleration.x;
    sample.accel(1) = accel.acceleration.y;
    sample.accel(2) = accel.acceleration.z;

    sample.gyro(0) = gyro.gyro.x;
    sample.gyro(1) = gyro.gyro.y;
    sample.gyro(2) = gyro.gyro.z;

    sample.timestamp = millis() * 0.001f;
    return sample;
}

IMUSample applyAlignment(const IMUSample& raw,
                         const IMUAlignment& align,
                         const Matrix<3,1>& gyro_body)
{
    IMUSample out;
    out.timestamp = raw.timestamp;

    // Rotate sensor frame into vehicle body frame
    out.gyro  = align.R * raw.gyro;
    out.accel = align.R * raw.accel;

    // Lever arm correction — removes centripetal acceleration caused by
    // the IMU being offset from the centre of mass.
    // a_corrected = a - w x (w x r)  =  a - (w*(w.r) - r*(w.w))
    // w_dot term dropped (low dynamics assumption, fine for a car)
    const Matrix<3,1>& w = gyro_body;
    const Matrix<3,1>& r = align.arm;

    float w_dot_r = w(0)*r(0) + w(1)*r(1) + w(2)*r(2);
    float w_dot_w = w(0)*w(0) + w(1)*w(1) + w(2)*w(2);

    out.accel(0) -= w(0)*w_dot_r - r(0)*w_dot_w;
    out.accel(1) -= w(1)*w_dot_r - r(1)*w_dot_w;
    out.accel(2) -= w(2)*w_dot_r - r(2)*w_dot_w;

    return out;
}

IMUSample fuseIMUs(const IMUSample& imu1_body, const IMUSample& imu2_body)
{
    // Thresholds — if sensors disagree beyond these, trust IMU1 (primary)
    // Tune these once you have real data; start conservative.
    const float ACCEL_THRESH = 1.5f;  // m/s^2
    const float GYRO_THRESH  = 0.1f;  // rad/s

    IMUSample out;
    out.timestamp = imu1_body.timestamp;

    for (int i = 0; i < 3; i++) {
        float da = fabsf(imu1_body.accel(i) - imu2_body.accel(i));
        float dg = fabsf(imu1_body.gyro(i)  - imu2_body.gyro(i));

        out.accel(i) = (da < ACCEL_THRESH)
            ? 0.5f * (imu1_body.accel(i) + imu2_body.accel(i))
            : imu1_body.accel(i);

        out.gyro(i) = (dg < GYRO_THRESH)
            ? 0.5f * (imu1_body.gyro(i) + imu2_body.gyro(i))
            : imu1_body.gyro(i);
    }
    return out;
}
