#include "ElementStorage.h"
#include <Arduino.h>
#include <ekf.h>


// Extended Kalman Filter
// What it does: predicts the next INS state with sensor inputs
// How it works: sensor data is fed into transition matrices, those matrices
// are then applied to the current INS state to give the next state.
//
// Requires gps lock to initialise on startup, those coordinates zero the vehicle.
//
// State vector (15 states):
//   [PX, PY, PZ, VX, VY, VZ, ROLL, PITCH, YAW, BAX, BAY, BAZ, BGX, BGY, BGZ]

EKF::EKF() {
    x_.Fill(0.0f);
    P_.Fill(0.0f);
    Q_.Fill(0.0f);
    for (int i = 0; i < STATE_SIZE; i++) {
        P_(i,i) = 1.0f;
        Q_(i,i) = 0.0001f; // tune as needed
    }
    // Clear cached values used by computePhi
    acc_world_cached_.Fill(0.0f);
    R_cached_.Fill(0.0f);
    for (int i = 0; i < 3; i++) R_cached_(i,i) = 1.0f;
}

void EKF::init(const StateVector& x0) {
    x_ = x0;
}

void EKF::getOrigin(float& lat, float& lng) const {
    lat = origin_lat_deg_;
    lng = origin_lng_deg_;
}

void EKF::setOrigin(float lat0_deg, float lng0_deg) {
    origin_lat_deg_ = lat0_deg;
    origin_lng_deg_ = lng0_deg;
    origin_set_     = true;
}

// Convert EKF local-frame meters back to absolute degrees
void EKF::getLatLng(float& lat_out, float& lng_out) const {
    // 1 degree latitude = ~111,111 m (constant)
    // 1 degree longitude = ~111,111 * cos(lat) m (shrinks toward poles)
    float cos_lat = cosf(origin_lat_deg_ * M_PI / 180.0f);

    lat_out = origin_lat_deg_ + (x_(PX) / 111111.0f);
    lng_out = origin_lng_deg_ + (x_(PY) / (111111.0f * cos_lat));
}

// ---- predict next state ----
void EKF::predict(const Matrix<3,1>& accel,
                  const Matrix<3,1>& gyro,
                  float dt)
{
    // Non-linear state prediction (also caches R and acc_world for Phi)
    x_ = processModel(x_, accel, gyro, dt);

    // Linearize using cached rotation matrix and body acceleration
    Covariance Phi = computePhi(dt);

    // Covariance propagation: P = Phi * P * Phi^T + Q
    P_ = Phi * P_ * ~Phi + Q_;

    // Enforce symmetry to prevent numerical drift
    P_ = (P_ + ~P_) * 0.5f;
}

void EKF::updateGPS(const Matrix<3,1>& gps_pos)
{
    Matrix<3,1> z = gps_pos;

    // h(x) = position states
    Matrix<3,1> h;
    h(0) = x_(PX);
    h(1) = x_(PY);
    h(2) = x_(PZ);

    // Innovation
    Matrix<3,1> y = z - h;

    // Observation matrix H (3 x STATE_SIZE)
    Matrix<3,STATE_SIZE> H;
    H.Fill(0.0f);
    H(0,PX) = 1.0f;
    H(1,PY) = 1.0f;
    H(2,PZ) = 1.0f;

    // GPS measurement noise (tune to your receiver's spec)
    Matrix<3,3> R;
    R.Fill(0.0f);
    R(0,0) = 2.0f; // x (m^2)
    R(1,1) = 2.0f; // y (m^2)
    R(2,2) = 4.0f; // z (m^2) — typically noisier

    // Innovation covariance and Kalman gain
    Matrix<3,3> S     = H * P_ * ~H + R;
    Matrix<3,3> S_inv = Invert(S);
    Matrix<STATE_SIZE,3> K = P_ * ~H * S_inv;

    // State update
    x_ = x_ + K * y;

    // Covariance update (Joseph form for numerical stability)
    Matrix<STATE_SIZE,STATE_SIZE> I;
    I.Fill(0.0f);
    for (int i = 0; i < STATE_SIZE; i++) I(i,i) = 1.0f;
    Matrix<STATE_SIZE,STATE_SIZE> IKH = I - K * H;
    P_ = IKH * P_ * ~IKH + K * R * ~K;

    // Enforce symmetry
    P_ = (P_ + ~P_) * 0.5f;
}

StateVector EKF::processModel(const StateVector& x,
                              const Matrix<3,1>& accel,
                              const Matrix<3,1>& gyro,
                              float dt)
{
    StateVector xn = x;

    // ---- Bias-corrected sensor readings ----
    Matrix<3,1> a;
    a(0) = accel(0) - x(BAX);
    a(1) = accel(1) - x(BAY);
    a(2) = accel(2) - x(BAZ);

    Matrix<3,1> w;
    w(0) = gyro(0) - x(BGX);
    w(1) = gyro(1) - x(BGY);
    w(2) = gyro(2) - x(BGZ);

    // ---- Rotation matrix: body -> world (ZYX convention) ----
    float cr = cosf(x(ROLL)),  sr = sinf(x(ROLL));
    float cp = cosf(x(PITCH)), sp = sinf(x(PITCH));
    float cy = cosf(x(YAW)),   sy = sinf(x(YAW));

    Matrix<3,3> R;
    R(0,0) = cy*cp;            R(0,1) = cy*sp*sr - sy*cr;  R(0,2) = cy*sp*cr + sy*sr;
    R(1,0) = sy*cp;            R(1,1) = sy*sp*sr + cy*cr;  R(1,2) = sy*sp*cr - cy*sr;
    R(2,0) = -sp;              R(2,1) = cp*sr;              R(2,2) = cp*cr;

    // Cache for computePhi (called immediately after this)
    R_cached_ = R;

    // ---- World-frame acceleration (subtract gravity) ----
    Matrix<3,1> acc_world = R * a;
    acc_world(2) -= 9.81f;
    acc_world_cached_ = acc_world;

    // ---- Integrate velocity ----
    xn(VX) += acc_world(0) * dt;
    xn(VY) += acc_world(1) * dt;
    xn(VZ) += acc_world(2) * dt;

    // ---- Integrate position (use updated velocity for better accuracy) ----
    xn(PX) += xn(VX) * dt;
    xn(PY) += xn(VY) * dt;
    xn(PZ) += xn(VZ) * dt;

    // ---- Euler rate kinematics (valid for small pitch — fine for a car) ----
    // ROLL_dot  = wx + (wy*sr + wz*cr) * tan(pitch)
    // PITCH_dot = wy*cr - wz*sr
    // YAW_dot   = (wy*sr + wz*cr) / cos(pitch)
    //
    // Guard against cos(pitch) ~= 0 (gimbal lock at ±90°, not reachable by car)
    float cp_safe = (fabsf(cp) > 0.01f) ? cp : 0.01f;
    float tan_p   = sp / cp_safe;

    xn(ROLL)  += (w(0) + (w(1)*sr + w(2)*cr) * tan_p) * dt;
    xn(PITCH) += (w(1)*cr - w(2)*sr) * dt;
    xn(YAW)   += (w(1)*sr + w(2)*cr) / cp_safe * dt;

    return xn;
}

// Build the linearised state-transition matrix (Jacobian of processModel)
// Uses rotation matrix and body acceleration cached during processModel.
Covariance EKF::computePhi(float dt)
{
    Covariance Phi;
    Phi.Fill(0.0f);
    for (int i = 0; i < STATE_SIZE; i++) Phi(i,i) = 1.0f;

    // ---- Position wrt velocity ----
    Phi(PX, VX) = dt;
    Phi(PY, VY) = dt;
    Phi(PZ, VZ) = dt;

    // ---- Velocity wrt attitude ----
    // dv/d(attitude) = -R * skew(a_body) * dt
    // For a car staying near level this is the dominant coupling term.
    // a_body is available from the cached world acceleration:
    //   acc_world = R * a_body  =>  a_body ≈ R^T * acc_world_cached
    // skew(a_body):
    //   [ 0    -az   ay ]
    //   [ az    0   -ax ]
    //   [-ay    ax    0 ]
    // Column j of  (-R * skew * dt)  goes into Phi(VX:VZ, attitude_j)
    // We use the cached R and acc_world to derive a_body = R^T * acc_world
    Matrix<3,1> a_body = ~R_cached_ * acc_world_cached_;
    float ax = a_body(0), ay = a_body(1), az = a_body(2);

    // -R * skew(a_body) columns (wrt ROLL, PITCH, YAW)
    // skew col for ROLL  (d/d_roll  of  R*a_body):  R * [0, az, -ay]^T
    // skew col for PITCH (d/d_pitch of  R*a_body):  R * [-az, 0, ax]^T
    // skew col for YAW   (d/d_yaw   of  R*a_body):  R * [ay, -ax, 0]^T
    Matrix<3,1> col_roll, col_pitch, col_yaw;
    col_roll(0) = 0.0f;  col_roll(1) =  az;   col_roll(2) = -ay;
    col_pitch(0) = -az;  col_pitch(1) = 0.0f;  col_pitch(2) =  ax;
    col_yaw(0) =  ay;   col_yaw(1) = -ax;   col_yaw(2) = 0.0f;

    Matrix<3,1> dv_droll  = R_cached_ * col_roll;
    Matrix<3,1> dv_dpitch = R_cached_ * col_pitch;
    Matrix<3,1> dv_dyaw   = R_cached_ * col_yaw;

    Phi(VX, ROLL)  = dv_droll(0)  * dt;
    Phi(VY, ROLL)  = dv_droll(1)  * dt;
    Phi(VZ, ROLL)  = dv_droll(2)  * dt;

    Phi(VX, PITCH) = dv_dpitch(0) * dt;
    Phi(VY, PITCH) = dv_dpitch(1) * dt;
    Phi(VZ, PITCH) = dv_dpitch(2) * dt;

    Phi(VX, YAW)   = dv_dyaw(0)   * dt;
    Phi(VY, YAW)   = dv_dyaw(1)   * dt;
    Phi(VZ, YAW)   = dv_dyaw(2)   * dt;

    // ---- Attitude wrt attitude (Euler kinematic Jacobian) ----
    // From the Euler rate equations used in processModel.
    // Only the small off-diagonal terms matter for a near-level vehicle;
    // the diagonal is already 1 from the identity initialisation above.
    float sr = sinf(x_(ROLL)), cr = cosf(x_(ROLL));
    float cp = cosf(x_(PITCH)), sp = sinf(x_(PITCH));
    float cp_safe  = (fabsf(cp) > 0.01f) ? cp : 0.01f;
    float tan_p    = sp / cp_safe;
    float sec_p    = 1.0f / cp_safe;

    // Bias-corrected gyro (approximation: use last state)
    float wy = x_(BGY);  // actual bias — sign cancels in Jacobian context
    float wz = x_(BGZ);
    // We want the Jacobian of [roll_dot, pitch_dot, yaw_dot] wrt [roll, pitch]
    // d(roll_dot)/d(roll)  = (wy*cr - wz*sr) * tan_p
    // d(roll_dot)/d(pitch) = (wy*sr + wz*cr) * sec_p^2
    // d(pitch_dot)/d(roll) = -wy*sr - wz*cr
    // d(yaw_dot)/d(roll)   = (wy*cr - wz*sr) * sec_p
    // d(yaw_dot)/d(pitch)  = (wy*sr + wz*cr) * sec_p * tan_p
    Phi(ROLL,  ROLL)  += (wy*cr - wz*sr) * tan_p * dt;
    Phi(ROLL,  PITCH) += (wy*sr + wz*cr) * sec_p*sec_p * dt;
    Phi(PITCH, ROLL)  += (-wy*sr - wz*cr) * dt;
    Phi(YAW,   ROLL)  += (wy*cr - wz*sr) * sec_p * dt;
    Phi(YAW,   PITCH) += (wy*sr + wz*cr) * sec_p * tan_p * dt;

    // ---- Bias states: identity (biases modelled as random walks via Q) ----
    // Already handled by the identity initialisation above.

    return Phi;
}
