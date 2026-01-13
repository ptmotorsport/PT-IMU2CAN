
#pragma once
#include <BasicLinearAlgebra.h>
using namespace BLA;

struct IMUSample {
    Matrix<3,1> accel;
    Matrix<3,1> gyro;
    float timestamp;
};

// Initialise in setup 
// TODO: make it take and address instead to ensure correct imu, ie generalise function into method. (done need to test in main)

bool initIMU_1(uint8_t address);

IMUSample readIMU(uint8_t address);
