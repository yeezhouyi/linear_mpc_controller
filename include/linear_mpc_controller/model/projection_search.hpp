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
// NOTE: the Python side's same-place foot collapse (R5/R6) and arc-
// contiguity run partition are ported here so the C++ mirror reproduces the
// SIGNED-IN golden on cusp / fold / straight_jump geometries.  The tie
// resolution is: (1) argmin dist2; (2) same-place feet (within 2*tie_eps)
// collapse to the best; (3) arc-contiguity run partition (span/median gap)
// -- a single run or same-direction runs collapse to the best; (4) genuine
// cross-direction runs resolve toward s_prev if present, else refuse (-2).
#ifndef LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_SEARCH_HPP_
#define LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_SEARCH_HPP_

#include <algorithm>
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

  // pick among candidate segment indices: faithful mirror of mpc_core.frenet
  // _pick -- argmin dist2, then the R5/R6 tie resolution.  Returns a segment
  // index in cand, or -2 on a genuine cross-lane tie with no s_prev state
  // (caller maps -2 / out-of-window to stage 3: refuse, fail-open closed).
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
    // j = argmin within the near band (first occurrence on ties)
    std::size_t j = 0;
    double jd = dist2[near_v[0]];
    for (std::size_t k = 1; k < near_v.size(); ++k) {
      if (dist2[near_v[k]] < jd) {
        jd = dist2[near_v[k]];
        j = k;
      }
    }
    // (1) same-place collapse: candidates whose feet coincide (within
    // 2*tie_eps) are the SAME physical point -- e.g. a cusp where the
    // forward and reverse strands meet.  Return the argmin best.
    {
      bool same_place = true;
      for (const auto i : near_v) {
        const double fdx = proj_x[i] - proj_x[near_v[j]];
        const double fdy = proj_y[i] - proj_y[near_v[j]];
        if (std::hypot(fdx, fdy) > 2.0 * tie_eps_m) {
          same_place = false;
          break;
        }
      }
      if (same_place) {
        cand = static_cast<int>(near_v[j]);
        return;
      }
    }
    // (2) arc-contiguity run partition: a single contiguous arc run (one
    // lane) collapses to the best; several runs separated by an arc gap are
    // genuinely different places/lanes.  Thresholds scale with the span and
    // the median intra-run spacing so a 2-strand tie is NOT misread as a
    // break (matches Python's lenient span/median gap logic).
    std::vector<double> arcs_n;
    arcs_n.reserve(near_v.size());
    for (const auto i : near_v) {
      arcs_n.push_back(traj[i].s + along[i]);
    }
    std::vector<std::size_t> order(arcs_n.size());
    for (std::size_t k = 0; k < order.size(); ++k) {
      order[k] = k;
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return arcs_n[a] < arcs_n[b]; });
    std::vector<double> sorted_arcs;
    sorted_arcs.reserve(arcs_n.size());
    for (const auto o : order) {
      sorted_arcs.push_back(arcs_n[o]);
    }
    std::vector<double> gap;
    for (std::size_t k = 1; k < sorted_arcs.size(); ++k) {
      gap.push_back(sorted_arcs[k] - sorted_arcs[k - 1]);
    }
    const double span = sorted_arcs.empty()
                          ? 0.0
                          : (sorted_arcs.back() - sorted_arcs.front());
    const double run_break_thr =
      std::max(0.05, 4.0 * span / static_cast<double>(std::max<std::size_t>(
                                    near_v.size() - 1, 1)));
    double med = 0.0;
    if (!gap.empty()) {
      std::vector<double> gs = gap;
      std::sort(gs.begin(), gs.end());
      if (gs.size() % 2 == 1) {
        med = gs[gs.size() / 2];
      } else {
        med = 0.5 * (gs[gs.size() / 2 - 1] + gs[gs.size() / 2]);
      }
    }
    const double break_thr = std::max(0.05, 5.0 * med);
    bool any_break = false;
    for (const double gv : gap) {
      if (gv > break_thr) {
        any_break = true;
        break;
      }
    }
    if (!any_break) {
      cand = static_cast<int>(near_v[j]);   // single run -> argmin best
      return;
    }
    // (3) multiple runs: keep one representative (argmin) per run, decide by
    // travel direction.  Mirror Python's order[rs:] slice (to end) so reps
    // match exactly.
    std::vector<std::size_t> run_starts;
    run_starts.push_back(0);
    for (std::size_t k = 0; k < gap.size(); ++k) {
      if (gap[k] > break_thr) {
        run_starts.push_back(k + 1);
      }
    }
    std::vector<std::size_t> reps;
    std::vector<double> rep_angles;
    for (std::size_t r = 0; r < run_starts.size(); ++r) {
      const std::size_t rs = run_starts[r];
      std::size_t bk = rs;
      double bd = dist2[near_v[order[rs]]];
      for (std::size_t k = rs + 1; k < order.size(); ++k) {
        const std::size_t seg = near_v[order[k]];
        if (dist2[seg] < bd) {
          bd = dist2[seg];
          bk = k;
        }
      }
      reps.push_back(near_v[order[bk]]);
      rep_angles.push_back(seg_yaw[near_v[order[bk]]]);
    }
    bool same_dir = true;
    for (std::size_t a = 0; a < rep_angles.size() && same_dir; ++a) {
      for (std::size_t b = a + 1; b < rep_angles.size(); ++b) {
        double dpsi = std::fmod(rep_angles[a] - rep_angles[b] + kPi,
                                2.0 * kPi);
        if (dpsi < 0.0) {
          dpsi += 2.0 * kPi;
        }
        dpsi -= kPi;
        if (std::fabs(dpsi) > 0.52) {   // ~30 deg: different direction
          same_dir = false;
          break;
        }
      }
    }
    if (same_dir) {
      cand = static_cast<int>(near_v[j]);   // same direction -> argmin
      return;
    }
    // (4) genuine cross-direction runs: resolve toward s_prev if we have
    // state, otherwise refuse (caller maps to stage 3).
    if (s_prev >= 0.0) {
      std::size_t best_tie = reps[0];
      double best_ds = std::fabs((traj[reps[0]].s + along[reps[0]]) - s_prev);
      for (std::size_t k = 1; k < reps.size(); ++k) {
        const double ds =
          std::fabs((traj[reps[k]].s + along[reps[k]]) - s_prev);
        if (ds < best_ds) {
          best_ds = ds;
          best_tie = reps[k];
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
