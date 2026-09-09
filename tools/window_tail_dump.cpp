// Cross-language parity dump for the LTV prediction window tail.
// Prints reference-state rows for a path whose head/tail curvature/speed
// differ; tools/check_window_tail_parity.py compares against the python
// reference (mpc_core.model.build_ltv_window).  Before the tail fix the C++
// rows past the trajectory end sampled the path START and diverged.
#include <cmath>
#include <cstdio>
#include <vector>

#include "linear_mpc_controller/mpc/qp_problem.hpp"

using namespace linear_mpc_controller;

namespace
{
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
    const double th = i * kDs / kR;
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
}  // namespace

int main()
{
  const auto traj = makePath();
  const double end = traj.back().s;
  MpcParams p;
  p.Ts = 0.05;
  p.N = 8;

  struct Case { const char * name; double base; };
  const Case cases[] = {{"mid", 1.0}, {"near", end - 0.10}, {"past", end + 0.30}};
  for (const auto & c : cases) {
    CondensedMpcProblem prob(p, traj, c.base);
    for (int k = 0; k < p.N; ++k) {
      const auto r = prob.referenceState(k);
      std::printf("%s %d %.12g %.12g\n", c.name, k, r(kV), r(kOmega));
    }
  }
  return 0;
}
