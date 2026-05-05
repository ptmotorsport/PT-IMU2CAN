#pragma once
#include "ElementStorage.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

// ---- State Definition ----
// x = [px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bgx, bgy, bgz]
static constexpr int STATE_SIZE = 15;

using StateVector = BLA::Matrix<STATE_SIZE,1>;
using Covariance = Matrix<STATE_SIZE, STATE_SIZE>;

class EKF {
public:
    EKF();

    enum StateIndex {
    PX = 0, PY = 1, PZ = 2,
    VX = 3, VY = 4, VZ = 5,
    ROLL = 6, PITCH = 7, YAW = 8,
    BAX = 9, BAY = 10, BAZ = 11, BGX = 12,
    BGY = 13, BGZ = 14
    };

    void init (const StateVector& x0);

    // Call once before first predict(), with your first valid GPS fix
    void setOrigin(float lat0_deg, float lng0_deg);

    void getLatLng(float& lat_out, float& lng_out) const;

    // IMU predition
    void predict(const Matrix<3,1>& accel,
                 const Matrix<3,1>& gyro,
                 float dt);

    // GPS position update
    void updateGPS(const Matrix<3,1>& gps_pos);

    const StateVector& getState() const {return x_;}
    const Covariance& covariance() const {return P_;}

    

private:
    float origin_lat_deg_ = 0.0f;
    float origin_lng_deg_ = 0.0f;
    bool  origin_set_     = false;

    StateVector x_;
    Covariance  P_;
    Covariance  Q_;

    Matrix<3,3> R_cached_;
    Matrix<3,1> acc_world_cached_;

    StateVector processModel(const StateVector& x,
          		   const Matrix<3,1>& accel,
          		   const Matrix<3,1>& gyro,
          		   float dt);

    Covariance computePhi(float dt);

    void updateAltimeter(float altitude_m);
};

template<int N>
Matrix<N,N> Identity()
{
    Matrix<N,N> I;
    I.Fill(0.0f);
    for (int i = 0; i < N; i++)
        I(i,i) = 1.0f;
    return I;
}
