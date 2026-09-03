// Self-contained C++ tests for the ROS-free core (U2/U4).  Compile & run on
// WSL2:  cmake -S . -B build && cmake --build build && ctest --test-dir build
// These do NOT require OSQP (solver-gated parts are covered by the Python
// reference tests until the OSQP backend is wired).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "linear_mpc_controller/mpc/fallback_policy.hpp"

using namespace linear_mpc_controller;

namespace
{
double wrapAngle_(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

std::vector<TrackPoint> straightLine(double length, double v, double ds)
{
  std::vector<TrackPoint> pts;
  for (double s = 0.0; s <= length + 1e-9; s += ds) {
    TrackPoint p;
    p.s = s; p.x = s; p.y = 0.0; p.yaw = 0.0; p.kappa = 0.0; p.v = v;
    pts.push_back(p);
  }
  return pts;
}
}  // namespace

static void test_linearisation()
{
  const double v_r = 0.8, kappa = 0.5, eps = 1e-7;
  Eigen::Matrix4d A;
  Eigen::Matrix<double, 4, 2> B;
  linearContinuousMatrices(v_r, kappa, A, B);
  // finite-difference of the nonlinear dynamics at the reference point
  const Eigen::Vector4d x0(0.0, 0.0, v_r, kappa * v_r);
  Eigen::Matrix4d J = Eigen::Matrix4d::Zero();
  for (int j = 0; j < 4; ++j) {
    Eigen::Vector4d xp = x0, xm = x0;
    xp(j) += eps; xm(j) -= eps;
    J.col(j) = (nonlinearErrorDerivative(xp, v_r, kappa) -
      nonlinearErrorDerivative(xm, v_r, kappa)) / (2.0 * eps);
  }
  assert((A - J).cwiseAbs().maxCoeff() < 1e-5);
  assert(B(2, 0) == 1.0 && B(3, 1) == 1.0);
  // equilibrium: affine term zero
  const Eigen::Vector4d f = nonlinearErrorDerivative(x0, v_r, kappa);
  assert(f.cwiseAbs().maxCoeff() < 1e-12);
  std::printf("test_linearisation OK\n");
}

static void test_discretisation()
{
  const double v_r = 0.6, kappa = 0.5, Ts = 0.05;
  Eigen::Matrix4d A;
  Eigen::Matrix<double, 4, 2> B;
  linearContinuousMatrices(v_r, kappa, A, B);
  Eigen::Matrix4d A_z;
  Eigen::Matrix<double, 4, 2> B_z;
  discretizeZoh(A, B, Ts, A_z, B_z);
  // high-rate integration of the linear continuous system
  const int n_sub = 2000;
  const double dt = Ts / n_sub;
  Eigen::Vector4d x0(0.05, 0.03, v_r - 0.05, kappa * v_r + 0.02);
  const Eigen::Vector2d u(0.2, 0.1);
  Eigen::Vector4d truth = x0;
  for (int i = 0; i < n_sub; ++i) truth += (A * truth + B * u) * dt;
  const Eigen::Vector4d pred = A_z * x0 + B_z * u;
  assert((pred - truth).norm() < 1e-4);
  std::printf("test_discretisation OK (zoh err %.2e)\n", (pred - truth).norm());
}

static void test_frenet_signs()
{
  const auto traj = straightLine(8.0, 0.8, 0.02);
  const FrenetProjection p0 = closestPoint(traj, 2.0, 0.4);
  assert(std::fabs(p0.e_y - 0.4) < 1e-9);   // left positive
  const FrenetProjection p1 = closestPoint(traj, 2.0, -0.3);
  assert(std::fabs(p1.e_y + 0.3) < 1e-9);
  assert(std::fabs(p0.arc - 2.0) < 1e-6);
  TrackPoint anchor;
  const Eigen::Vector4d err = frenetState(traj, 10.0, 0.2, 0.0, 0.8, 0.0, 0.0, anchor);
  assert(std::fabs(anchor.s - traj.back().s) < 1e-6);  // clamps at the end
  assert(std::fabs(err(0) - 0.2) < 1e-9);
  std::printf("test_frenet_signs OK\n");
}

static void test_fallback()
{
  FallbackParams fp;
  fp.Ts = 0.05;
  fp.stop_deceleration = 0.8;
  fp.max_hold_cycles = 6;
  FallbackPolicy fb(fp);
  double v, w;
  HealthState h;
  int stage;
  // critical -> immediate zero
  fb.apply(0.8, 0.3, HealthState::NO_REFERENCE, v, w, h, stage);
  assert(v == 0.0 && w == 0.0 && stage == 3);
  // NaN -> zero
  fb.apply(std::nan(""), 1.0, HealthState::OK, v, w, h, stage);
  assert(v == 0.0 && h == HealthState::NAN_OUTPUT);
  // healthy pass-through
  fb.apply(0.5, 0.2, HealthState::OK, v, w, h, stage);
  assert(v == 0.5 && stage == 0);
  // QP failure -> degrade then emergency
  int n = 0;
  double last = 1.0;
  for (; n < 20; ++n) {
    fb.apply(last, 0.0, HealthState::QP_INFEASIBLE, v, w, h, stage);
    if (h == HealthState::EMERGENCY_STOP) { assert(v == 0.0); break; }
    assert(v <= last + 1e-12);
    last = v;
  }
  assert(n <= fp.max_hold_cycles + 1);
  std::printf("test_fallback OK (emergency after %d cycles)\n", n + 1);
}

int main()
{
  test_linearisation();
  test_discretisation();
  test_frenet_signs();
  test_fallback();
  std::printf("all C++ core tests passed\n");
  return 0;
}
