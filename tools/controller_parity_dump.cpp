// Deterministic complete-controller parity fixture for the Python repair tree.
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "linear_mpc_controller/mpc/linear_mpc.hpp"

using namespace linear_mpc_controller;

namespace
{
constexpr double kPiLocal = 3.14159265358979323846;

MpcParams params()
{
  MpcParams p;
  p.Ts = 0.05;
  p.N = 25;
  p.v_max = 0.30;
  p.omega_max = 1.20;
  p.a_max = 0.65;
  p.alpha_max = 2.0;
  p.qp_timeout_s = 0.0;  // parity fixture is numerical, not a deadline test
  return p;
}

void addStraight(std::vector<TrackPoint> & out)
{
  for (int i = 0; i <= 120; ++i) {
    const double s = 0.05 * i;
    out.push_back(TrackPoint{s, s, 0.0, 0.0, 0.0, 0.15});
  }
}

void addArc(std::vector<TrackPoint> & out, bool left)
{
  const double radius = 3.0;
  const double sign = left ? 1.0 : -1.0;
  for (int i = 0; i <= 120; ++i) {
    const double theta = 1.2 * static_cast<double>(i) / 120.0;
    const double s = radius * theta;
    const double x = radius * std::sin(theta);
    const double y = sign * radius * (1.0 - std::cos(theta));
    out.push_back(TrackPoint{s, x, y, sign * theta, sign / radius, 0.15});
  }
}

void addSCurve(std::vector<TrackPoint> & out)
{
  const double length = 6.0;
  const double amplitude = 0.25;
  const double w = 2.0 * kPiLocal / length;
  for (int i = 0; i <= 240; ++i) {
    const double s = length * static_cast<double>(i) / 240.0;
    const double dy = amplitude * w * std::cos(w * s);
    const double ddy = -amplitude * w * w * std::sin(w * s);
    const double yaw = std::atan(dy);
    const double kappa = ddy / std::pow(1.0 + dy * dy, 1.5);
    out.push_back(TrackPoint{s, s, amplitude * std::sin(w * s), yaw, kappa, 0.15});
  }
}

struct Fixture
{
  const char * name;
  std::vector<TrackPoint> trajectory;
  double x;
  double y;
  double yaw;
  double v;
  double omega;
};

std::vector<Fixture> fixtures()
{
  std::vector<Fixture> result;
  std::vector<TrackPoint> straight;
  addStraight(straight);
  result.push_back({"straight", straight, 0.80, 0.04, 0.02, 0.15, 0.01});

  std::vector<TrackPoint> left;
  addArc(left, true);
  const double tl = 0.315;
  result.push_back({"left_arc", left,
    3.0 * std::sin(tl) - 0.03 * std::sin(tl),
    3.0 * (1.0 - std::cos(tl)) + 0.03 * std::cos(tl),
    tl + 0.02, 0.15, 0.15 / 3.0 + 0.01});

  std::vector<TrackPoint> right;
  addArc(right, false);
  const double tr = 0.315;
  result.push_back({"right_arc", right,
    3.0 * std::sin(tr) + 0.03 * std::sin(tr),
    -3.0 * (1.0 - std::cos(tr)) - 0.03 * std::cos(tr),
    -tr - 0.02, 0.15, -0.15 / 3.0 - 0.01});

  std::vector<TrackPoint> s_curve;
  addSCurve(s_curve);
  result.push_back({"s_curve", s_curve, 2.4, 0.0, 0.0, 0.15, 0.0});
  return result;
}

}  // namespace

int main()
{
  // Build the S-curve state with the same analytic tangent as addSCurve.
  auto fs = fixtures();
  const double length = 6.0;
  const double amplitude = 0.25;
  const double w = 2.0 * kPiLocal / length;
  const double s = 2.4;
  const double dy = amplitude * w * std::cos(w * s);
  const double yaw = std::atan(dy);
  const double nx = -std::sin(yaw);
  const double ny = std::cos(yaw);
  fs.back().x = s + 0.03 * nx;
  fs.back().y = amplitude * std::sin(w * s) + 0.03 * ny;
  fs.back().yaw = yaw + 0.02;
  fs.back().omega = (-amplitude * w * w * std::sin(w * s) /
    std::pow(1.0 + dy * dy, 1.5)) * 0.15 + 0.01;

  std::cout << std::setprecision(17);
  for (const auto & f : fs) {
    LinearMpcController controller(params(), f.trajectory);
    const auto out = controller.computeCycle(f.x, f.y, f.yaw, f.v, f.omega);
    std::cout << f.name << "," << static_cast<int>(out.health) << ","
      << static_cast<int>(out.qp_status) << "," << out.a0 << "," << out.alpha0
      << "," << out.v_cmd << "," << out.omega_cmd << ","
      << out.e_used(0) << "," << out.e_used(1) << "," << out.e_used(2)
      << "," << out.e_used(3) << "," << out.accepted_arc << "\n";
  }
  return 0;
}
