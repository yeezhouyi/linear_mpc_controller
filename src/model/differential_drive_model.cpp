#include "linear_mpc_controller/model/differential_drive_model.hpp"

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

/// Dense matrix exponential (scaling & squaring + Taylor), Eigen based.
template <int N>
Eigen::Matrix<double, N, N> expmDense(const Eigen::Matrix<double, N, N> & M)
{
  const double norm = M.cwiseAbs().rowwise().sum().maxCoeff();
  int s = 0;
  if (norm > 1.0) s = static_cast<int>(std::ceil(std::log2(norm)));
  Eigen::Matrix<double, N, N> A = M / static_cast<double>(1 << s);
  Eigen::Matrix<double, N, N> acc = Eigen::Matrix<double, N, N>::Identity();
  Eigen::Matrix<double, N, N> term = Eigen::Matrix<double, N, N>::Identity();
  for (int k = 1; k <= 16; ++k) {
    term = term * A / static_cast<double>(k);
    acc += term;
  }
  for (int i = 0; i < s; ++i) acc = acc * acc;
  return acc;
}

}  // namespace

Eigen::Vector4d nonlinearErrorDerivative(const Eigen::Vector4d & x, double v_r, double kappa)
{
  (void)v_r;
  const double e_y = x[kEy];
  const double e_psi = x[kEpsi];
  const double v = x[kV];
  const double omega = x[kOmega];
  double denom = 1.0 - kappa * e_y;
  if (std::abs(denom) < 1e-9) denom = denom >= 0.0 ? 1e-9 : -1e-9;
  Eigen::Vector4d d;
  d << v * std::sin(e_psi), omega - kappa * v * std::cos(e_psi) / denom, 0.0, 0.0;
  return d;
}

void linearContinuousMatrices(double v_r, double kappa, Eigen::Matrix4d & A,
  Eigen::Matrix<double, 4, 2> & B)
{
  A.setZero();
  A(kEy, kEpsi) = v_r;
  A(kEpsi, kEy) = -kappa * kappa * v_r;
  A(kEpsi, kV) = -kappa;
  A(kEpsi, kOmega) = 1.0;
  B.setZero();
  B(kV, kA) = 1.0;
  B(kOmega, kAlpha) = 1.0;
}

void discretizeZoh(const Eigen::Matrix4d & A, const Eigen::Matrix<double, 4, 2> & B,
  double Ts, Eigen::Matrix4d & A_d, Eigen::Matrix<double, 4, 2> & B_d)
{
  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  M.block<4, 4>(0, 0) = A;
  M.block<4, 2>(0, 4) = B;
  const Eigen::Matrix<double, 6, 6> E = expmDense<6>(M * Ts);
  A_d = E.block<4, 4>(0, 0);
  B_d = E.block<4, 2>(0, 4);
}

void discretizeEuler(const Eigen::Matrix4d & A, const Eigen::Matrix<double, 4, 2> & B,
  double Ts, Eigen::Matrix4d & A_d, Eigen::Matrix<double, 4, 2> & B_d)
{
  A_d = Eigen::Matrix4d::Identity() + Ts * A;
  B_d = Ts * B;
}

FrenetProjection closestPoint(const std::vector<TrackPoint> & traj, double px, double py)
{
  FrenetProjection best;
  double best_d2 = std::numeric_limits<double>::infinity();
  const int n = static_cast<int>(traj.size());
  for (int i = 0; i + 1 < n; ++i) {
    const TrackPoint & p0 = traj[i];
    const TrackPoint & p1 = traj[i + 1];
    double dx = p1.x - p0.x, dy = p1.y - p0.y;
    double len = std::hypot(dx, dy);
    double tx = 0.0, ty = 0.0;
    if (len > 1e-12) { tx = dx / len; ty = dy / len; }
    else { tx = std::cos(p0.yaw); ty = std::sin(p0.yaw); }
    double along = (px - p0.x) * tx + (py - p0.y) * ty;
    along = std::max(0.0, std::min(along, len));
    double pjx = p0.x + along * tx, pjy = p0.y + along * ty;
    double d2 = (pjx - px) * (pjx - px) + (pjy - py) * (pjy - py);
    if (d2 < best_d2) {
      best_d2 = d2;
      best.seg = i;
      best.w = len > 1e-12 ? along / len : 0.0;
      best.e_y = (px - pjx) * (-ty) + (py - pjy) * tx;  // left normal (-ty, tx)
      best.arc = p0.s + along;
    }
  }
  return best;
}

TrackPoint sampleByArc(const std::vector<TrackPoint> & traj, double s)
{
  const int n = static_cast<int>(traj.size());
  if (n == 0) return TrackPoint{};
  if (s <= traj.front().s) return traj.front();
  if (s >= traj.back().s) return traj.back();
  int idx = 0;
  for (int i = 0; i < n - 1; ++i) {
    if (traj[i].s <= s && s <= traj[i + 1].s) { idx = i; break; }
  }
  const TrackPoint & a = traj[idx];
  const TrackPoint & b = traj[idx + 1];
  double denom = b.s - a.s;
  double w = denom > 1e-12 ? (s - a.s) / denom : 0.0;
  TrackPoint out;
  out.s = a.s + w * (b.s - a.s);
  out.x = a.x + w * (b.x - a.x);
  out.y = a.y + w * (b.y - a.y);
  double dyaw = wrapAngle(b.yaw - a.yaw);
  out.yaw = wrapAngle(a.yaw + w * dyaw);
  out.kappa = a.kappa + w * (b.kappa - a.kappa);
  out.v = a.v + w * (b.v - a.v);
  return out;
}

Eigen::Vector4d frenetState(const std::vector<TrackPoint> & traj, double px, double py,
  double yaw, double v, double omega, double lookahead_m, TrackPoint & anchor)
{
  const FrenetProjection proj = closestPoint(traj, px, py);
  const double base_arc = proj.arc;
  const double arc = std::min(base_arc + lookahead_m, traj.back().s);
  anchor = sampleByArc(traj, arc);
  Eigen::Vector4d err;
  err << proj.e_y, wrapAngle(yaw - anchor.yaw), v, omega;
  return err;
}

}  // namespace linear_mpc_controller
