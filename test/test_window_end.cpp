// Regression: the LTV prediction window must HOLD the trajectory end once
// the preview crosses it (mirror of mpc_core/model.py build_ltv_window's
// "pad the last arc at the end"), instead of leaving zero-initialised
// entries that sample the path START.  The path here has a straight head
// (v=0.4, kappa=0) and a curved tail (v=1.0, kappa=1.0), so a tail that
// wraps back to s=0 is easy to tell apart from a tail that holds the end.
#include <cmath>
#include <cstdio>
#include <vector>

#include "linear_mpc_controller/mpc/qp_problem.hpp"

using namespace linear_mpc_controller;

namespace
{
// straight v=0.4 kappa=0 for s in [0,2], then left arc radius 1 kappa=1
// v=1.0 for s in (2, 3.5].  Geometry shared with tools/window_tail_dump.cpp
// and tools/check_window_tail_parity.py (cross-language parity).
constexpr double kDs = 0.01;
constexpr double kL1 = 2.0;
constexpr double kR = 1.0;
constexpr double kAngle = 1.5;

std::vector<TrackPoint> makePath()
{
  std::vector<TrackPoint> pts;
  for (int i = 0; i <= static_cast<int>(kL1 / kDs); ++i) {
    TrackPoint p;
    p.s = i * kDs;
    p.x = p.s;
    p.y = 0.0;
    p.yaw = 0.0;
    p.kappa = 0.0;
    p.v = 0.4;
    pts.push_back(p);
  }
  const int n_arc = static_cast<int>(kAngle / kDs);
  for (int i = 1; i <= n_arc; ++i) {
    const double th = i * kDs / kR;   // = arc length / R
    TrackPoint p;
    p.s = kL1 + i * kDs;
    p.x = kL1 + kR * std::sin(th);
    p.y = kR - kR * std::cos(th);
    p.yaw = th;
    p.kappa = 1.0 / kR;
    p.v = 1.0;
    pts.push_back(p);
  }
  return pts;
}

void expectClose(double a, double b, double tol, const char * what, int & fails)
{
  if (std::fabs(a - b) > tol) {
    std::printf("FAIL %s: %.9g != %.9g\n", what, a, b);
    ++fails;
  }
}
}  // namespace

int main()
{
  int fails = 0;
  const auto traj = makePath();
  if (traj.empty() || std::fabs(traj.back().s - (kL1 + kAngle * kR)) > 1e-9) {
    std::printf("FAIL path length\n");
    return 1;
  }
  MpcParams p;
  p.Ts = 0.05;
  p.N = 8;
  const double end = traj.back().s;
  const double end_v = traj.back().v;              // 1.0
  const double end_om = traj.back().kappa * traj.back().v;  // 1.0
  const double start_v = 0.4;                      // kappa 0 -> om 0
  const double tol = 1e-9;

  // Case A: normal mid-path window (no end crossing) must be unchanged.
  {
    CondensedMpcProblem prob(p, traj, 1.0);
    for (int k = 0; k < p.N; ++k) {
      const auto r = prob.referenceState(k);
      char buf[96];
      std::snprintf(buf, sizeof(buf), "mid k=%d v", k);
      expectClose(r(kV), start_v, tol, buf, fails);
      std::snprintf(buf, sizeof(buf), "mid k=%d om", k);
      expectClose(r(kOmega), 0.0, tol, buf, fails);
    }
  }
  // Case B: base near the end -- the tail MUST hold the end point.
  {
    CondensedMpcProblem prob(p, traj, end - 0.10);
    for (int k = 1; k < p.N; ++k) {   // k=0 sits just before the end
      const auto r = prob.referenceState(k);
      char buf[96];
      std::snprintf(buf, sizeof(buf), "near k=%d v", k);
      expectClose(r(kV), end_v, tol, buf, fails);
      std::snprintf(buf, sizeof(buf), "near k=%d om", k);
      expectClose(r(kOmega), end_om, tol, buf, fails);
    }
  }
  // Case C: base past the end -- every sample holds the end point.
  {
    CondensedMpcProblem prob(p, traj, end + 0.30);
    for (int k = 0; k < p.N; ++k) {
      const auto r = prob.referenceState(k);
      char buf[96];
      std::snprintf(buf, sizeof(buf), "past k=%d v", k);
      expectClose(r(kV), end_v, tol, buf, fails);
      std::snprintf(buf, sizeof(buf), "past k=%d om", k);
      expectClose(r(kOmega), end_om, tol, buf, fails);
    }
  }

  if (fails) {
    std::printf("window_end: %d FAILURES\n", fails);
    return 1;
  }
  std::printf("window_end: OK\n");
  return 0;
}
