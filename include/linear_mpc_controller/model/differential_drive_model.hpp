// Differential-drive / Frenet error model -- ROS-free core (U2).
//
// Written to match the authoritative derivation in docs/mpc_model_derivation.md
// and the verified Python reference (mpc_core/model.py, mpc_core/frenet.py).
//
// NOTE: this tree is compiled & tested on WSL2 (colcon); the Windows dev host
// has no C++ toolchain.  Math parity is covered by the Python reference tests.
#ifndef LINEAR_MPC_CONTROLLER__MODEL__DIFFERENTIAL_DRIVE_MODEL_HPP_
#define LINEAR_MPC_CONTROLLER__MODEL__DIFFERENTIAL_DRIVE_MODEL_HPP_

#include <Eigen/Dense>
#include <vector>

namespace linear_mpc_controller
{

// Frozen dimensions / conventions (docs/mpc_model_derivation.md)
constexpr int kStateDim = 4;   // [e_y, e_psi, v, omega]
constexpr int kInputDim = 2;   // [a, alpha]
constexpr int kEy = 0, kEpsi = 1, kV = 2, kOmega = 3;
constexpr int kA = 0, kAlpha = 1;

/// One dense reference point (s, x, y, yaw, kappa, v).
struct TrackPoint
{
  double s = 0.0, x = 0.0, y = 0.0, yaw = 0.0, kappa = 0.0, v = 0.0;
  double omega() const { return kappa * v; }
};

/// Continuous Frenet error dynamics:
///   d[e_y]    = v sin(e_psi)
///   d[e_psi]  = omega - kappa*v*cos(e_psi)/(1 - kappa*e_y)
///   d[v] = 0, d[omega] = 0   (accel enters through B)
Eigen::Vector4d nonlinearErrorDerivative(
  const Eigen::Vector4d & x, double v_r, double kappa);

/// Analytic LTV matrices about (0, 0, v_r, kappa*v_r).
void linearContinuousMatrices(double v_r, double kappa, Eigen::Matrix4d & A, Eigen::Matrix<double, 4, 2> & B);

/// ZOH discretisation via the augmented matrix exponential.
void discretizeZoh(const Eigen::Matrix4d & A, const Eigen::Matrix<double, 4, 2> & B,
  double Ts, Eigen::Matrix4d & A_d, Eigen::Matrix<double, 4, 2> & B_d);

/// First-order Euler (kept for error comparison only).
void discretizeEuler(const Eigen::Matrix4d & A, const Eigen::Matrix<double, 4, 2> & B,
  double Ts, Eigen::Matrix4d & A_d, Eigen::Matrix<double, 4, 2> & B_d);

/// Frenet projection of a pose onto a dense trajectory (left-positive e_y).
/// Returns segment index, interpolation weight, signed lateral error and arc.
struct FrenetProjection
{
  int seg = 0;
  double w = 0.0;
  double e_y = 0.0;
  double arc = 0.0;
};

FrenetProjection closestPoint(
  const std::vector<TrackPoint> & traj, double px, double py);

/// Error state x = [e_y, e_psi, v, omega] w.r.t. anchor at arc+lookahead.
/// Out: anchor (reference point used this cycle).
Eigen::Vector4d frenetState(
  const std::vector<TrackPoint> & traj,
  double px, double py, double yaw, double v, double omega,
  double lookahead_m, TrackPoint & anchor);

/// Sample reference at a given arc by linear interpolation (clamped).
TrackPoint sampleByArc(const std::vector<TrackPoint> & traj, double s);

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MODEL__DIFFERENTIAL_DRIVE_MODEL_HPP_
