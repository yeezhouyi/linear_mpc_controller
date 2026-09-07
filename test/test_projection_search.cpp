// A2 C++ mirror tests: closestPointWindowed (see projection_search.hpp).
// Mirrors mpc_core/tests/test_a2_heading_gate.py on agreeing cases.
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "linear_mpc_controller/model/projection_search.hpp"

using linear_mpc_controller::WindowedProjection;
using linear_mpc_controller::TrackPoint;
using linear_mpc_controller::closestPointWindowed;
using linear_mpc_controller::kPi;

static double kNaN()
{
  return std::numeric_limits<double>::quiet_NaN();
}

static std::vector<TrackPoint> straight(double length, double ds = 0.02)
{
  std::vector<TrackPoint> t;
  const int n = static_cast<int>(length / ds) + 1;
  for (int i = 0; i < n; ++i) {
    TrackPoint p;
    p.s = i * ds;
    p.x = i * ds;
    p.y = 0.0;
    p.yaw = 0.0;
    p.v = 0.5;
    t.push_back(p);
  }
  return t;
}

/// Circle centred at (0,0), CCW (start (r,0) heading +pi/2), ds spacing.
static std::vector<TrackPoint> circle(double r, double ds = 0.02)
{
  std::vector<TrackPoint> t;
  const int cn = static_cast<int>((2.0 * kPi * r) / ds);
  for (int i = 0; i <= cn; ++i) {
    const double th = (static_cast<double>(i) * ds) / r;
    TrackPoint p;
    p.x = r * std::cos(th);
    p.y = r * std::sin(th);
    p.yaw = th + kPi / 2.0;
    p.v = 0.5;
    t.push_back(p);
  }
  for (std::size_t i = 1; i < t.size(); ++i) {
    const double d = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
    t[i].s = t[i - 1].s + d;
  }
  return t;
}

static bool approx(double a, double b, double tol = 1e-6)
{
  return std::fabs(a - b) < tol;
}

int main()
{
  // -- forward-aligned pose on a straight: accepted, exact arc/e_y ----------
  {
    auto tr = straight(8.0);
    WindowedProjection r = closestPointWindowed(tr, 1.0, 0.05, 0.0, 0.98);
    if (!(r.stage < 3 && approx(r.arc, 1.0, 0.03) && approx(r.e_y, 0.05, 1e-6))) {
      std::printf("FAIL forward pose stage=%d arc=%.6f e_y=%.6f\n",
                  r.stage, r.arc, r.e_y);
      return 1;
    }
  }
  // -- heading gate rejects every segment -> stage 3 (fail-open closed) -----
  {
    auto tr = straight(8.0);
    WindowedProjection r = closestPointWindowed(tr, 1.0, 0.0, kPi, 0.5);
    if (r.stage != 3) {
      std::printf("FAIL all-reject: stage=%d (expected 3)\n", r.stage);
      return 1;
    }
  }
  // -- reverse travel via uniform gear override is accepted -----------------
  {
    auto tr = straight(8.0);
    WindowedProjection r =
      closestPointWindowed(tr, 1.0, 0.0, kPi, 0.98, {}, -1);
    if (!(r.stage < 3 && approx(r.arc, 1.0, 0.03))) {
      std::printf("FAIL reverse gear: stage=%d arc=%.6f\n", r.stage, r.arc);
      return 1;
    }
  }
  // -- mid-circle interior pose: windowed hit, exact-ish arc ---------------
  {
    auto tr = circle(2.0);
    const double arc_want = 2.99;
    const double th = arc_want / 2.0;
    const double px = 2.0 * std::cos(th), py = 2.0 * std::sin(th);
    const double yaw = th + kPi / 2.0;
    WindowedProjection r = closestPointWindowed(tr, px, py, yaw, arc_want - 0.1);
    if (!(r.stage < 3 && approx(r.arc, arc_want, 0.04))) {
      std::printf("FAIL circle: stage=%d arc=%.6f (want ~%.2f)\n",
                  r.stage, r.arc, arc_want);
      return 1;
    }
  }
  // -- with s_prev, a near-vertex pose resolves deterministically ----------
  // (stateless on a 0.02 m-sampled dense path legitimately reports stage 3
  // under doc semantics -- the Python side's same-lane collapse refinement
  // is the recorded divergence, see projection_search.hpp header note)
  {
    auto tr = straight(8.0);
    WindowedProjection r =
      closestPointWindowed(tr, 2.05, 0.05, 0.0, 2.0);
    if (!(r.stage < 3 && approx(r.arc, 2.05, 0.03))) {
      std::printf("FAIL s_prev resolve: stage=%d arc=%.6f\n", r.stage, r.arc);
      return 1;
    }
  }
  std::printf("all closestPointWindowed tests passed\n");
  return 0;
}
