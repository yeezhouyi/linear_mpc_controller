#include "trajectory_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace linear_mpc_controller
{

namespace
{
double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}
}  // namespace

std::vector<TrackPoint> adaptPath(const nav_msgs::msg::Path & path, const AdapterParams & p)
{
  const size_t n = path.poses.size();
  if (n < 2) return {};

  std::vector<TrackPoint> pts(n);
  // 1) tangent from neighbour chords
  for (size_t i = 0; i < n; ++i) {
    size_t j0 = i == 0 ? 0 : i - 1;
    size_t j1 = i == n - 1 ? n - 1 : i + 1;
    const auto & a = path.poses[j0].pose.position;
    const auto & b = path.poses[j1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    pts[i].yaw = (j1 == j0) ? 0.0 : std::atan2(dy, dx);
  }
  // 2) arc length (cumulative chord)
  double s = 0.0;
  pts[0].s = 0.0;
  for (size_t i = 1; i < n; ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    s += std::hypot(b.x - a.x, b.y - a.y);
    pts[i].s = s;
  }
  // 3) curvature from heading differences over 2-chord arc
  for (size_t i = 1; i + 1 < n; ++i) {
    const double d_prev = pts[i].s - pts[i - 1].s;
    const double d_next = pts[i + 1].s - pts[i].s;
    const double arc = 0.5 * (d_prev + d_next);
    pts[i].kappa = arc > 1e-12 ? wrapAngle(pts[i + 1].yaw - pts[i - 1].yaw) / (2.0 * arc) : 0.0;
  }
  pts[0].kappa = n > 1 ? pts[1].kappa : 0.0;
  pts[n - 1].kappa = n > 1 ? pts[n - 2].kappa : 0.0;
  // 4) positions + speed completion (deterministic)
  for (size_t i = 0; i < n; ++i) {
    pts[i].x = path.poses[i].pose.position.x;
    pts[i].y = path.poses[i].pose.position.y;
    const double k = std::fabs(pts[i].kappa);
    double v = p.v_default;
    if (k > 1e-6) v = std::min(v, p.curve_speed / std::max(k, 1e-6));
    pts[i].v = std::min(v, p.v_max);
  }
  return pts;
}

}  // namespace linear_mpc_controller
