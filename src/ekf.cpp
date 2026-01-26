#include "ElementStorage.h"
#include <ekf.h>

EKF::EKF() {
    x_.Fill(0.0f);
    P_.Fill(0.0f);
    Q_.Fill(0.0f);

    for (int i = 0; i < STATE_SIZE; i++) {
      P_(i,i) = 1.0f;
      Q_(i,i) = 0.0001f; // tune needed
    }
}

void EKF::init(const StateVector& x0) {
    x_ = x0;
}

// ---- predict next state ----
void EKF::predict(const Matrix<3,1>& accel,
                  const Matrix<3,1>& gyro,
                  float dt) 
{
    // non-linear prediction 
    x_ = processModel(x_, accel, gyro, dt);
  
    // linearize
    Covariance Phi = computePhi(dt);
  
    // covariance propagation
    P_ = Phi * P_ * -Phi + Q_;
}

void EKF::updateGPS(const Matrix<3,1>& gps_pos) 
{
    // copy deref to z
    Matrix<3,1> z = gps_pos;
 
    // h(x) = position
    Matrix<3,1> h;
    h(0) = x_(0);
    h(1) = x_(1);
    h(2) = x_(2);
 
    // differnce 
    Matrix<3,1> y = z - h;

 
    // H matrix (3x15)
    Matrix<3,STATE_SIZE> H;
    H.Fill(0.0f);
 
    H(0,0) = 1.0f;
    H(1,1) = 1.0f;
    H(2,2) = 1.0f;
 
    // Measurement noise
    Matrix<3,3> R;
    R.Fill(0.0f);
    R(0,0) = 2.0f;
    R(1,1) = 2.0f;
    R(2,2) = 4.0f;
 
    // Kalman gain calculation
    Matrix<3,3> S = H * P_ * ~H + R;
    Matrix<3,3> S_inv = Invert(S);
 
    Matrix<STATE_SIZE,3> K = P_ * ~H * S_inv;
 
    // state update
    x_ = x_ + K * y;
 
    // covariance update
    Matrix<STATE_SIZE, STATE_SIZE> I;
    I.Fill(0.0f);
    for (int i = 0; i < STATE_SIZE; i++) I(i,i) = 1.0f;
 
    P_ = (I - K * H) * P_;
}


StateVector EKF::processModel(const StateVector& x,
                              const Matrix<3,1>& accel,
                              const Matrix<3,1>& gyro,
                              float dt)
{
    StateVector xn = x;

    // position += velocity * dt
    xn(0) += x(3) * dt;
    xn(1) += x(4) * dt;
    xn(2) += x(5) * dt;

    // velocity += accel * dt (placeholder)
    xn(3) += accel(0) * dt;
    xn(4) += accel(1) * dt;
    xn(5) += accel(2) * dt;

    // orientation += gyro * dt
    xn(6) += gyro(0) * dt;
    xn(7) += gyro(1) * dt;
    xn(8) += gyro(2) * dt;

    return xn;
}

Covariance EKF::computePhi(float dt)
{
    Covariance Phi;
    Phi.Fill(0.0f);

    for (int i = 0; i < STATE_SIZE; i++)
        Phi(i,i) = 1.0f;

    // position wrt velocity
    Phi(0,3) = dt;
    Phi(1,4) = dt;
    Phi(2,5) = dt;

    return Phi;
}
