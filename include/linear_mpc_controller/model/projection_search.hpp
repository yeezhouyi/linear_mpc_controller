// A2 C++ mirror: windowed + heading-gate closest-point projection.
// Mirrors the 4060-doc A2 semantics implemented in mpc_core/frenet.py:
//   * s_prev window escalation: stage 0 narrow [s-back, s+fwd], stage 1 wide
//     [s+-wide] (candidates also gated by distance <= reacquire), stage 2
//     global;
//   * heading gate at every stage: motion direction from the EXPLICIT
//     per-segment gear (+1/-1; never inferred from reference speed), or the
//     test-only uniform travel_sign override;
//   * stage 3 when the gate rejects every segment or a genuine no-s_prev tie
//     exists -- NO unconstrained global-argmin fallback (fail-open closed);
//   * ties within tie_eps resolve toward s_prev in arc.
// NOTE (recorded): the Python side later added same-lane micro-segment
// collapse refinements for 0.02 m-sampled dense paths; this C++ mirror keeps
// the doc semantics and is exercised on poses where the two agree (off-vertex
// interiors and gate cases); parity of the FULL Python refinements is a
// tracked A5B follow-up.
#ifndef LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_SEARCH_HPP_
#define LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_SEARCH_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"

namespace linear_mpc_controller
{

constexpr double kPi = 3.14159265358979323846;

/// Result of a windowed projection (seg index into N-1 segments; stage 0..3).
struct WindowedProjection
{
  int seg = -1;              // -1 when stage == 3
  double w = 0.0;            // interpolation weight in [0,1]
  double e_y = std::numeric_limits<double>::quiet_NaN();
  double arc = 0.0;
  int stage = 3;             // 0 narrow-window / 1 wide / 2 global / 3 ambiguous
};

/// Segment geometry precomputed once per call.
struct SegmentGeo
{
  std::vector<double> len, tx, ty, seg_s;   // seg_s: arc at each segment start
  double total = 0.0;
};

inline void buildSegmentGeo(const std::vector<TrackPoint> & traj, SegmentGeo & g)
{
  const std::size_t n = traj.size();
  const std::size_t m = n - 1;
  g.len.assign(m, 0.0);
  g.tx.assign(m, 0.0);
  g.ty.assign(m, 0.0);
  g.seg_s.assign(m, 0.0);
  g.total = n ? traj.back().s : 0.0;
  for (std::size_t i = 0; i < m; ++i) {
    const double dx = traj[i + 1].x - traj[i].x;
    const double dy = traj[i + 1].y - traj[i].y;
    const double L = std::hypot(dx, dy);
    g.len[i] = L;
    g.seg_s[i] = traj[i].s;
    if (L > 1e-12) {
      g.tx[i] = dx / L;
      g.ty[i] = dy / L;
    } else if (i > 0) {
      g.tx[i] = g.tx[i - 1];   // degenerate: carry previous tangent
      g.ty[i] = g.ty[i - 1];
    }
  }
}

/// Windowed closest-point projection (A2 semantics; see header comment).
/// yaw: robot heading, or quiet_NaN to disable the heading gate.
/// s_prev: last accepted arc, or negative to disable windows (global only).
/// gear: per-segment +1/-1 (size m) or empty => all +1.
/// travel_sign: test-only uniform gear override; 0 => use gear array.
inline WindowedProjection closestPointWindowed(
  const std::vector<TrackPoint> & traj, double px, double py,
  double yaw, double s_prev,
  const std::vector<double> & gear = {}, int travel_sign = 0,
  double back_m = 0.08, double fwd_m = 0.30, double reacquire_m = 1.00,
  double wide_m = 5.00, double heading_gate_rad = kPi / 2.0,
  double tie_eps_m = 0.05)
{
  WindowedProjection out;
  const std::size_t n = traj.size();
  if (n < 2) {
    return out;   // stage 3
  }
  const std::size_t m = n - 1;
  SegmentGeo g;
  buildSegmentGeo(traj, g);

  // projection of the pose onto each segment (clamped feet)
  std::vector<double> along(m), proj_x(m), proj_y(m), dist2(m), seg_yaw(m);
  for (std::size_t i = 0; i < m; ++i) {
    const double rel_x = px - traj[i].x;
    const double rel_y = py - traj[i].y;
    double a = rel_x * g.tx[i] + rel_y * g.ty[i];
    a = std::min(std::max(a, 0.0), g.len[i]);
    along[i] = a;
    proj_x[i] = traj[i].x + a * g.tx[i];
    proj_y[i] = traj[i].y + a * g.ty[i];
    const double ddx = proj_x[i] - px;
    const double ddy = proj_y[i] - py;
    dist2[i] = ddx * ddx + ddy * ddy;
    seg_yaw[i] = std::atan2(g.ty[i], g.tx[i]);
  }

  // heading gate: motion direction from gear, never from reference speed
  std::vector<bool> ok(m, true);
  const bool gate_on = !std::isnan(yaw);
  if (gate_on) {
    const double motion_yaw =
      (travel_sign != 0)
        ? (travel_sign < 0 ? yaw + kPi : yaw)
        : yaw;
    for (std::size_t i = 0; i < m; ++i) {
      double sgn = 1.0;
      if (travel_sign == 0 && !gear.empty() && i < gear.size()) {
        sgn = gear[i];
      }
      const double myaw = (sgn > 0.0) ? motion_yaw : yaw + kPi;
      double dpsi = std::fmod(myaw - seg_yaw[i] + kPi, 2.0 * kPi);
      if (dpsi < 0.0) {
        dpsi += 2.0 * kPi;
      }
      dpsi -= kPi;
      ok[i] = std::fabs(dpsi) <= heading_gate_rad;
    }
  }

  // pick among candidate segment indices: min dist2, tie -> nearest s_prev
  auto pick = [&](const std::vector<std::size_t> & idx, int & cand) -> void {
    cand = -1;
    if (idx.empty()) {
      return;
    }
    std::size_t best_i = idx[0];
    double best_d2 = dist2[best_i];
    for (const auto i : idx) {
      if (dist2[i] < best_d2) {
        best_d2 = dist2[i];
        best_i = i;
      }
    }
    const double thr = std::pow(std::sqrt(best_d2) + tie_eps_m, 2.0);
    std::vector<std::size_t> near_v;
    for (const auto i : idx) {
      if (dist2[i] <= thr) {
        near_v.push_back(i);
      }
    }
    if (near_v.size() == 1) {
      cand = static_cast<int>(near_v[0]);
      return;
    }
    // RECORDED: same-lane collapse (mirror of the Python _pick refinement).
    // On dense (0.02 m) samplings the eps band always contains the same-lane
    // micro-segments around the foot (their clamped feet lie within tie_eps
    // of the best distance), so a raw tie resolution would lag the anchor by
    // up to one sample forever.  Candidates whose tangent differs from the
    // best by <= 0.05 rad collapse to the single best; only genuinely
    // different-lane candidates (antiparallel / neighbouring lanes) reach
    // the s_prev / stage-3 path.
    const double lane_rad = 0.05;
    std::vector<std::size_t> cross;
    for (const auto i : near_v) {
      double dpsi = std::fmod(seg_yaw[i] - seg_yaw[best_i] + kPi, 2.0 * kPi);
      if (dpsi < 0.0) {
        dpsi += 2.0 * kPi;
      }
      dpsi -= kPi;
      if (std::fabs(dpsi) > lane_rad) {
        cross.push_back(i);
      }
    }
    if (cross.empty()) {
      cand = static_cast<int>(best_i);   // single lane: argmin best
      return;
    }
    near_v.clear();
    near_v.push_back(best_i);            // keep the same-lane best + cross lanes
    for (const auto i : cross) {
      near_v.push_back(i);
    }
    if (s_prev >= 0.0) {
      // resolve toward s_prev in arc (cross-lane tie with state)
      std::size_t best_tie = near_v[0];
      double best_ds = std::fabs((traj[near_v[0]].s + along[near_v[0]]) - s_prev);
      for (const auto i : near_v) {
        const double ds = std::fabs((traj[i].s + along[i]) - s_prev);
        if (ds < best_ds) {
          best_ds = ds;
          best_tie = i;
        }
      }
      cand = static_cast<int>(best_tie);
      return;
    }
    cand = -2;   // genuine cross-lane tie with no state: caller maps to stage 3
  };

  // window escalation (only when s_prev given)
  int seg = -1, stage = 2;
  if (s_prev >= 0.0) {
    const double r2 = reacquire_m * reacquire_m;
    const double ws[2][2] = {{back_m, fwd_m}, {wide_m, wide_m}};
    for (int st = 0; st < 2; ++st) {
      const double lo = s_prev - ws[st][0];
      const double hi = s_prev + ws[st][1];
      std::vector<std::size_t> in_win;
      for (std::size_t i = 0; i < m; ++i) {
        if (ok[i] && traj[i + 1].s >= lo && traj[i].s <= hi) {
          in_win.push_back(i);
        }
      }
      int cand = -1;
      pick(in_win, cand);
      if (cand >= 0 && dist2[static_cast<std::size_t>(cand)] <= r2) {
        seg = cand;
        stage = st;
        break;
      }
    }
  }
  if (seg < 0) {
    std::vector<std::size_t> all;
    for (std::size_t i = 0; i < m; ++i) {
      if (ok[i]) {
        all.push_back(i);
      }
    }
    int cand = -1;
    pick(all, cand);
    if (cand == -2 || cand < 0) {
      // genuine no-s_prev tie or heading gate rejected everything: refuse
      // (NO unconstrained global-argmin fallback -- fail-open closed)
      out.stage = 3;
      out.arc = (s_prev >= 0.0) ? s_prev : 0.0;
      return out;
    }
    seg = cand;
    stage = 2;
  }

  const std::size_t s = static_cast<std::size_t>(seg);
  const double nx_ = -g.ty[s], ny_ = g.tx[s];   // left normal
  out.seg = seg;
  out.w = (g.len[s] > 1e-12) ? along[s] / g.len[s] : 0.0;
  out.w = std::min(std::max(out.w, 0.0), 1.0);
  out.e_y = (px - proj_x[s]) * nx_ + (py - proj_y[s]) * ny_;
  out.arc = g.seg_s[s] + along[s];
  out.stage = stage;
  return out;
}

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_SEARCH_HPP_
