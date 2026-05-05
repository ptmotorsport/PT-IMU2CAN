#pragma once
#include <BasicLinearAlgebra.h>
using namespace BLA;

struct IMUSample {
    Matrix<3,1> accel;
    Matrix<3,1> gyro;
    float timestamp;
};

struct IMUAlignment {
    Matrix<3,3> R;    // rotation from sensor frame -> vehicle body frame
    Matrix<3,1> arm;  // lever arm: position of IMU relative to CoM (metres)
};

// ---- Initialise in setup() ----
bool initIMU(uint8_t address);

// ---- Read raw sample from sensor ----
IMUSample readIMU(uint8_t address);

// ---- Rotate sample into body frame and remove lever arm effect ----
// gyro_body should be the already-aligned gyro of the primary IMU
IMUSample applyAlignment(const IMUSample& raw,
                         const IMUAlignment& align,
                         const Matrix<3,1>& gyro_body);

// ---- Fuse two body-frame samples, rejecting disagreeing axes ----
IMUSample fuseIMUs(const IMUSample& imu1_body, const IMUSample& imu2_body);
