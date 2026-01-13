#pragma once
#include "ElementStorage.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

// ---- State Definition ----
// x = [px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bgy, bgz]
static constexpr int STATE_SIZE = 15;

using StateVector = BLA::Matrix<STATE_SIZE,1>;
using Covariance = Matrix<STATE_SIZE, STATE_SIZE>;

class EKF {
public:
  EKF();

  void init (const StateVector& x0);

  // IMU predition
  void predict(const Matrix<3,1>& accel,
	       const Matrix<3,1>& gyro,
	       float dt);

  // GPS position update
  void updateGPS(const Matrix<3,1>& gps_pos);

  const StateVector& state() const {return x_;}
  const Covariance& covariance() const {return P_;}

private:
  StateVector x_;
  Covariance  P_;
  Covariance  Q_;

  StateVector processModel(const StateVector& x,
			   const Matrix<3,1>& accel,
			   const Matrix<3,1>& gyro,
			   float dt);

  Covariance computePhi(float dt);

  void updateAltimeter(float altitude_m);
};
