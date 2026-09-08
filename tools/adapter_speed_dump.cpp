// Dump the C++ Path->TrackPoint speed completion for a synthetic circle so
// the python reference completion can be compared against it (cross-language
// parity test for the Day 4-5 kinematic guard).
//
// Usage: adapter_speed_dump <curve_speed> <omega_max>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "trajectory_adapter.hpp"

int main(int argc, char ** argv)
{
  const double curve_speed = argc > 1 ? std::atof(argv[1]) : 2.0;
  const double omega_max = argc > 2 ? std::atof(argv[2]) : 2.0;

  // Circle of radius 0.10 m -> |kappa| ~ 10; with v_default 0.5 the unguarded
  // completion would need 5.0 rad/s (> omega_max = 2.0), so the guard must
  // cap v to ~0.2 m/s.
  const double R = 0.10;
  const int N = 72;
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i < N; ++i) {
    const double th = 2.0 * M_PI * static_cast<double>(i) / N;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "odom";
    ps.pose.position.x = R * std::cos(th);
    ps.pose.position.y = R * std::sin(th);
    path.poses.push_back(ps);
  }

  linear_mpc_controller::AdapterParams p;
  p.curve_speed = curve_speed;
  p.omega_max = omega_max;
  const auto pts = linear_mpc_controller::adaptPath(path, p);
  const double ratio = linear_mpc_controller::maxOmegaRatio(pts, omega_max);
  std::printf("curve_speed %.6f omega_max %.6f\n", curve_speed, omega_max);
  std::printf("max_omega_ratio %.9f\n", ratio);
  std::printf("points %zu\n", pts.size());
  for (const auto & q : pts) {
    std::printf("%.9f %.9f\n", q.kappa, q.v);
  }
  return 0;
}
