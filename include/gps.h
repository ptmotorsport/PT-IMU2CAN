#pragma once
#include <Arduino.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

struct GPSSample {
    Matrix<3,1> position;
    float speed_mps;
    int satellites;
    bool valid;
};

bool initGPS();

GPSSample readGPS();
