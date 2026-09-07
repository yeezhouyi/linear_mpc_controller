// A7.2 #5 cross-language projection parity dump (C++ side).
// Reads a pose file (lines: "x y yaw") + a trajectory spec (straight|circle
// [length/radius ds]) and chains the WINDOWED projection exactly like the
// Python controller-ledger path: s_prev = previous raw arc (kept on stage 3),
// yaw + gear(+1) given; prints per pose: "stage arc seg e_y".
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "linear_mpc_controller/model/projection_search.hpp"

using linear_mpc_controller::TrackPoint;
using linear_mpc_controller::WindowedProjection;
using linear_mpc_controller::closestPointWindowed;
using linear_mpc_controller::kPi;

static std::vector<TrackPoint> straight(double length, double ds)
{
  std::vector<TrackPoint> t;
  const int n = static_cast<int>(length / ds) + 1;
  for (int i = 0; i < n; ++i) {
    TrackPoint p;
    p.s = i * ds; p.x = i * ds; p.y = 0.0; p.yaw = 0.0; p.v = 0.5;
    t.push_back(p);
  }
  return t;
}

static std::vector<TrackPoint> circle(double r, double ds)
{
  std::vector<TrackPoint> t;
  const int cn = static_cast<int>((2.0 * kPi * r) / ds);
  for (int i = 0; i <= cn; ++i) {
    const double th = (static_cast<double>(i) * ds) / r;
    TrackPoint p;
    p.x = r * std::cos(th); p.y = r * std::sin(th);
    p.yaw = th + kPi / 2.0; p.v = 0.5;
    t.push_back(p);
  }
  for (std::size_t i = 1; i < t.size(); ++i) {
    const double d = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
    t[i].s = t[i - 1].s + d;
  }
  return t;
}

/// Fold: out along +x at y=0 to length, back at y=offset (arc monotone).
static std::vector<TrackPoint> fold(double length, double offset, double ds)
{
  std::vector<TrackPoint> t;
  const int n = static_cast<int>(length / ds) + 1;
  for (int i = 0; i < n; ++i) {
    TrackPoint p;
    p.s = i * ds; p.x = i * ds; p.y = 0.0; p.yaw = 0.0; p.v = 0.5;
    t.push_back(p);
  }
  double s = t.back().s;
  for (int i = n - 1; i >= 0; --i) {
    TrackPoint p;
    p.x = t[i].x; p.y = offset; p.yaw = kPi; p.v = 0.5;
    p.s = s + (t[n - 1].s - t[i].s);
    t.push_back(p);
  }
  return t;
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr,
      "usage: %s straight|circle|fold <poses.txt> <out.txt> [param ds]\n", argv[0]);
    return 2;
  }
  const std::string kind = argv[1];
  const double ds = argc >= 5 ? std::atof(argv[4]) : 0.02;
  std::vector<TrackPoint> traj;
  if (kind == "straight") {
    traj = straight(8.0, ds);
  } else if (kind == "circle") {
    traj = circle(2.0, ds);
  } else if (kind == "fold") {
    traj = fold(2.0, 0.30, ds);
  } else {
    std::fprintf(stderr, "unknown kind %s\n", kind.c_str());
    return 2;
  }

  FILE * fin = std::fopen(argv[2], "r");
  FILE * fout = std::fopen(argv[3], "w");
  if (!fin || !fout) {
    std::fprintf(stderr, "cannot open files\n");
    return 2;
  }
  double s_prev = -1.0;
  double x = 0.0, y = 0.0, yaw = 0.0;
  while (std::fscanf(fin, "%lf %lf %lf", &x, &y, &yaw) == 3) {
    WindowedProjection r = closestPointWindowed(traj, x, y, yaw, s_prev);
    std::fprintf(fout, "%d %.9f %d %.9f\n", r.stage, r.arc, r.seg,
                 std::isnan(r.e_y) ? -999.0 : r.e_y);
    if (r.stage != 3) {
      s_prev = r.arc;
    }
  }
  std::fclose(fin);
  std::fclose(fout);
  std::printf("parity dump done (%s)\n", kind.c_str());
  return 0;
}
