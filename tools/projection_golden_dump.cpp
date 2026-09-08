// A7.2 #5 / A5B.4: C++ consumer of the SIGNED-IN projection golden file.
//
// The golden (mpc_core/tests/data/projection_golden.json) is the single
// source of truth for the projection sequence: this tool replays the same
// pose sequences through the C++ windowed projection + A5.1 gate chain and
// prints one record per pose, so tools/check_projection_golden.py can diff
// C++ against the committed records (and against the Python reference).
//
// Kinds mirror mpc_core/tools/generate_projection_golden.py:
//   straight_jump  straight 6.0 m @ ds=0.02, lateral 0.05, 3 m teleport
//   fold           antiparallel lanes y=0 / y=0.30
//   cusp           +1 -> 0 -> -1 gear reversal at x=2
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "linear_mpc_controller/model/projection_gate.hpp"
#include "linear_mpc_controller/model/projection_search.hpp"

using linear_mpc_controller::kPi;
using linear_mpc_controller::ProjectionGateState;
using linear_mpc_controller::TrackPoint;
using linear_mpc_controller::WindowedProjection;
using linear_mpc_controller::closestPointWindowed;
using linear_mpc_controller::sampleByArc;

namespace
{

// ---- golden geometry defaults (mirror of the Python generator) -------------
constexpr double kFwdM = 0.30;
constexpr double kBackM = 0.08;
constexpr double kReacquireM = 1.00;
constexpr double kWideM = 5.00;
constexpr double kHeadingGateRad = 1.5708;
constexpr double kTieEpsM = 0.05;
constexpr double kCaptureM = 0.30;   // capture-resolve radius (A5.2)

std::vector<TrackPoint> straightTraj(double length = 6.0, double ds = 0.02)
{
  std::vector<TrackPoint> t;
  const int n = static_cast<int>(std::lround(length / ds)) + 1;
  for (int i = 0; i < n; ++i) {
    TrackPoint p;
    p.s = i * ds; p.x = i * ds; p.y = 0.0; p.yaw = 0.0; p.v = 0.6;
    t.push_back(p);
  }
  return t;
}

std::vector<TrackPoint> foldTraj(double length = 2.0, double offset = 0.30, int n = 101)
{
  // linspace arithmetic mirror: np.linspace(a,b,n)[i] = i*step + a with
  // step = (b-a)/(n-1).  The bit-exact operation order matters: the golden
  // tie-resolution is sensitive to 1-ulp differences of the foot position
  // (e.g. 2.0*i/(n-1) rounds differently from i*(2.0/(n-1)) on the forward
  // cusp strand, which flips the same-place argmin to the other strand).
  const double step = (length - 0.0) / static_cast<double>(n - 1);
  const double rstep = (0.0 - length) / static_cast<double>(n - 1);
  std::vector<TrackPoint> t;
  t.reserve(2 * static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {          // out: +x at y = 0
    TrackPoint p;
    p.x = static_cast<double>(i) * step + 0.0;
    p.y = 0.0; p.yaw = 0.0; p.v = 0.5;
    t.push_back(p);
  }
  for (int i = 0; i < n; ++i) {          // back: -x at y = offset
    TrackPoint p;
    p.x = static_cast<double>(i) * rstep + length;
    p.y = offset; p.yaw = kPi; p.v = 0.5;
    t.push_back(p);
  }
  for (std::size_t i = 1; i < t.size(); ++i) {
    t[i].s = t[i - 1].s + std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
  }
  return t;
}

std::vector<TrackPoint> cuspTraj(int n = 101)
{
  // linspace arithmetic mirror (see foldTraj comment): forward
  // linspace(0,2,n), reverse linspace(2,0,n), cumulative |dx| arc.
  const double step = 2.0 / static_cast<double>(n - 1);
  const double rstep = -2.0 / static_cast<double>(n - 1);
  std::vector<TrackPoint> t;
  t.reserve(2 * static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {          // forward to x = 2
    TrackPoint p;
    p.x = static_cast<double>(i) * step + 0.0;
    p.y = 0.0; p.yaw = 0.0; p.v = 0.8;
    t.push_back(p);
  }
  for (int i = 0; i < n; ++i) {          // reverse back to x = 0
    TrackPoint p;
    p.x = static_cast<double>(i) * rstep + 2.0;
    p.y = 0.0; p.yaw = 0.0; p.v = -0.5;
    t.push_back(p);
  }
  for (std::size_t i = 1; i < t.size(); ++i) {
    t[i].s = t[i - 1].s + std::fabs(t[i].x - t[i - 1].x);
  }
  return t;
}

/// gear for the cusp: +1 on the forward segments, -1 on the reverse ones.
std::vector<double> cuspGear(int n = 101)
{
  std::vector<double> g;
  g.reserve(2 * static_cast<std::size_t>(n) - 1);
  g.insert(g.end(), n - 1, 1.0);
  g.insert(g.end(), n, -1.0);
  return g;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr,
      "usage: %s straight_jump|fold|cusp <poses.txt> <out.txt>\n", argv[0]);
    return 2;
  }
  const std::string kind = argv[1];
  std::vector<TrackPoint> traj;
  std::vector<double> gear;
  if (kind == "straight_jump") {
    traj = straightTraj();
  } else if (kind == "fold") {
    traj = foldTraj();
  } else if (kind == "cusp") {
    traj = cuspTraj();
    gear = cuspGear();
  } else {
    std::fprintf(stderr, "unknown kind %s\n", kind.c_str());
    return 2;
  }

  std::ifstream fin(argv[2]);
  std::ofstream fout(argv[3]);
  if (!fin || !fout) {
    std::fprintf(stderr, "cannot open files\n");
    return 2;
  }

  // Parameter header: the checker diffs these against the golden header, so
  // a silent drift of the C++ window/gate defaults trips the test.
  fout << "# params back_m=" << kBackM << " fwd_m=" << kFwdM
       << " reacquire_m=" << kReacquireM << " wide_m=" << kWideM
       << " heading_gate_rad=" << kHeadingGateRad << " tie_eps_m=" << kTieEpsM
       << " projection_margin_m=" << linear_mpc_controller::kProjectionMarginM
       << " allowance_cap_m=" << linear_mpc_controller::kAllowanceCapM
       << " odom_step_noise_m=" << linear_mpc_controller::kOdomStepNoiseM
       << "\n";
  fout << std::setprecision(9);

  ProjectionGateState gate(1.5, 0.05);   // MpcParams().v_max / Ts
  bool first = true;
  double s_prev = -1.0;
  double x = 0.0, y = 0.0, yaw = 0.0;
  while (fin >> x >> y >> yaw) {
    if (first) {
      // prime the gate with the start pose (no heading gate, global search)
      const WindowedProjection a0 = closestPointWindowed(
        traj, x, y, std::numeric_limits<double>::quiet_NaN(), -1.0, gear, 0,
        kBackM, kFwdM, kReacquireM, kWideM, kHeadingGateRad, kTieEpsM);
      gate.reset(a0.arc);
      gate.step(x, y, a0.arc, sampleByArc(traj, gate.accepted_arc).yaw);
      first = false;
    }
    const WindowedProjection r = closestPointWindowed(
      traj, x, y, yaw, s_prev, gear, 0,
      kBackM, kFwdM, kReacquireM, kWideM, kHeadingGateRad, kTieEpsM);
    const auto dec = gate.step(
      x, y, r.arc, sampleByArc(traj, gate.accepted_arc).yaw);
    if (r.stage != 3) {
      const TrackPoint ap = sampleByArc(traj, r.arc);
      const bool near = std::hypot(x - ap.x, y - ap.y) <= kCaptureM;
      if (!dec.accepted && near) {
        gate.rebaseline(r.arc);          // capture-resolve (A5.2)
      }
    }
    if (r.stage != 3) {
      s_prev = r.arc;
    }
    fout << r.stage << " " << r.arc << " " << r.seg << " "
         << (std::isnan(r.e_y) ? -999.0 : r.e_y) << " "
         << gate.accepted_arc << "\n";
  }
  std::printf("golden dump done (%s)\n", kind.c_str());
  return 0;
}
