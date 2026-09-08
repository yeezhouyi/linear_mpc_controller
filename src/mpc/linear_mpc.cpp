#include "linear_mpc_controller/mpc/linear_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace linear_mpc_controller
{

LinearMpcController::LinearMpcController(const MpcParams & params, std::vector<TrackPoint> traj)
: params_(params), traj_(std::move(traj)), fallback_(FallbackParams{
    params_.Ts, params_.v_min, params_.v_max, params_.omega_max,
    0.8, 10})
{
  solver_ = makeDefaultSolver(params_.qp_max_iter, params_.qp_abs_tol, params_.qp_rel_tol);
  // A4.2: the protocol reads the same declared numbers as Python's MpcParams.
  reacq_.p.max_reject_run = params_.max_reject_run;
  reacq_.p.v_probation = params_.v_probation;
  reacq_.p.probation_steps = params_.probation_steps;
  reacq_.p.reacquire_stable_steps = params_.reacquire_stable_steps;
  reacq_.p.reacquire_timeout_steps = params_.reacquire_timeout_steps;
}

void LinearMpcController::setReference(std::vector<TrackPoint> traj)
{
  traj_ = std::move(traj);
  fallback_.reset();
  have_warm_ = false;
  // A5.1: a new Path resets the gate (never carry a stale anchor across).
  gate_.reset(0.0);
  gate_initialized_ = false;
  reject_run_ = 0;
  // A4.2: a new Path drops any in-flight reacquire (mirrors set_reference).
  reacq_.reset();
}

MpcCycleResult LinearMpcController::computeCycle(double px, double py, double yaw, double v, double omega)
{
  ++cycle_;
  MpcCycleResult res;
  if (traj_.size() < 2) {
    res.health = HealthState::NO_REFERENCE;
    res.reason = "no reference set";
    res.fallback_used = true;
    return res;
  }

  // ---- A4.2 reacquire seeking: no QP while we re-anchor ---------------
  // Mirrors mpc.py: GLOBAL search with the heading gate ON; candidates must
  // survive the four screens (distance / heading / ambiguity / same-seg
  // stability) before an atomic commit; every seeking cycle decelerates to
  // zero instead of feeding a stale anchor to the QP.
  if (reacq_.inSeeking()) {
    const WindowedProjection cand = closestPointWindowed(
      traj_, px, py, yaw, -1.0, {}, 0,
      params_.back_m, params_.fwd_m, params_.reacquire_m, params_.wide_m,
      params_.heading_gate_rad, params_.tie_eps_m);
    const ReacquireCycle rc = reacq_.seek(cand.seg, cand.arc, cand.stage);
    res.reacquire_seeking = true;
    res.reacquire_count = rc.reacquire_count;
    res.in_probation = rc.in_probation;
    res.projection_stage = cand.stage;
    res.accepted_arc = gate_.accepted_arc;
    if (rc.lost) {
      res.health = HealthState::EMERGENCY_STOP;
      res.reason = "PROJECTION_LOST";
      res.fallback_used = true;
      return res;
    }
    if (rc.committed) {
      // atomic commit (A4.2 steps 4/5): new baseline, zeroed budget,
      // dropped QP warm start -- then probation.
      gate_.reset(rc.committed_arc);
      gate_initialized_ = true;
      reject_run_ = 0;
      have_warm_ = false;
      res.accepted_arc = gate_.accepted_arc;
    }
    res.health = HealthState::OK;
    res.reason = "REACQUIRE_SEEKING";
    return res;   // decel-to-zero: zero command while seeking
  }

  // ---- A2 raw windowed projection + A5.1 acceptance gate --------------
  const double s_prev = gate_initialized_ ? gate_.accepted_arc : -1.0;
  const WindowedProjection raw = closestPointWindowed(
    traj_, px, py, yaw, s_prev, {}, 0,
    params_.back_m, params_.fwd_m, params_.reacquire_m, params_.wide_m,
    params_.heading_gate_rad, params_.tie_eps_m);
  res.projection_stage = raw.stage;
  if (raw.stage == 3) {
    // NO unconstrained global-argmin fallback (fail-open closed, A2).
    res.health = HealthState::EMERGENCY_STOP;
    res.reason = "PROJECTION_AMBIGUOUS";
    res.fallback_used = true;
    res.accepted_arc = gate_.accepted_arc;
    return res;
  }
  if (!gate_initialized_) {
    // anchor cycle: accept the first trustworthy projection (controller
    // semantics: baseline = raw arc unconditionally, mirroring mpc.py).
    gate_.reset(raw.arc);
    gate_initialized_ = true;
    reject_run_ = 0;
  } else {
    // tangent at the last ACCEPTED arc (never the raw candidate's segment)
    const TrackPoint tan_pt = sampleByArc(traj_, gate_.accepted_arc);
    const GateDecision dec = gate_.step(px, py, raw.arc, tan_pt.yaw);
    if (!dec.accepted) {
      ++reject_run_;
    } else {
      reject_run_ = 0;
    }
    if (reject_run_ > params_.max_reject_run) {
      // A4.2: stop trusting the stale anchor and enter SEEKING.  mpc.py does
      // NOT return on this cycle -- the QP still runs on the frozen baseline
      // and the NEXT cycle decelerates to zero while seeking.
      reacq_.enterSeeking();
    }
  }
  // ---- A4.2 probation window (A5.0): speed cap, completion says no ----
  const bool probation = reacq_.tickProbation();
  res.in_probation = probation;
  res.reacquire_count = reacq_.reacquire_events;
  // anchor/err at the ACCEPTED arc (A5.0): the QP never sees a rejected
  // raw projection.
  TrackPoint anchor;
  Eigen::Vector4d x0;
  frenetErrorAtArc(px, py, yaw, v, omega, anchor, x0);
  res.e_used = x0;
  res.accepted_arc = gate_.accepted_arc;
  res.reject_run = reject_run_;

  CondensedMpcProblem prob(params_, traj_, anchor.s);
  Eigen::MatrixXd H, C;
  Eigen::VectorXd q, l, u;
  prob.build(x0, H, q, C, l, u);

  QpSolution sol = solver_->solve(H, q, C, l, u, have_warm_ ? warm_ : Eigen::VectorXd());
  res.qp_status = sol.status;
  res.qp_iterations = sol.iterations;
  res.qp_time_us = sol.solve_time_us;

  if (sol.status == QpSolution::Status::kSolved ||
      sol.status == QpSolution::Status::kApproximate) {
    // shift warm start: drop first block, pad zeros
    warm_ = Eigen::VectorXd::Zero(2 * params_.N);
    if (2 * params_.N > 2) {
      warm_.head(2 * params_.N - 2) = sol.u.tail(2 * params_.N - 2);
    }
    have_warm_ = true;
    const double a0 = sol.u(0);
    const double alpha0 = sol.u(1);
    double v_cmd = x0(kV) + params_.Ts * a0;
    const double w_cmd = x0(kOmega) + params_.Ts * alpha0;
    if (reject_run_ > 0) {
      v_cmd = std::min(v_cmd, params_.v_probation);   // A5.0 reject cap
    }
    res.constraint_violation = maxConstraintViolation(x0, prob, sol.u);

    int stage = 0;
    HealthState h = HealthState::OK;
    fallback_.apply(v_cmd, w_cmd, HealthState::OK, res.v_cmd, res.omega_cmd, h, stage);
    res.health = h;
    res.fallback_used = stage >= 2;
    if (h != HealthState::OK) res.reason = "clamped/degraded by fallback";
    return res;
  }

  // solver failure -> fallback from zero (deterministic stop)
  int stage = 0;
  HealthState h = HealthState::QP_INFEASIBLE;
  fallback_.apply(0.0, 0.0, HealthState::QP_INFEASIBLE, res.v_cmd, res.omega_cmd, h, stage);
  res.health = h;
  res.reason = "qp failed";
  res.fallback_used = true;
  have_warm_ = false;
  return res;
}

double LinearMpcController::maxConstraintViolation(const Eigen::Vector4d & x0,
  const CondensedMpcProblem & prob, const Eigen::VectorXd & U) const
{
  const int N = params_.N;
  double viol = 0.0;
  Eigen::Vector4d x = x0;
  for (int k = 0; k < N; ++k) {
    x = prob.A_d()[k] * x + prob.B_d()[k] * U.segment<2>(2 * k);
    viol = std::max({viol, params_.v_min - x(kV), x(kV) - params_.v_max,
      std::fabs(x(kOmega)) - params_.omega_max});
  }
  return std::max(viol, 0.0);
}

void LinearMpcController::frenetErrorAtArc(
  double px, double py, double yaw, double v, double omega,
  TrackPoint & anchor, Eigen::Vector4d & err) const
{
  double arc = gate_.accepted_arc;
  if (params_.lookahead_m > 0.0) {
    arc = std::min(arc + params_.lookahead_m, traj_.back().s);
  }
  arc = std::min(std::max(arc, 0.0), traj_.back().s);
  anchor = sampleByArc(traj_, arc);
  const double e_y = (px - anchor.x) * (-std::sin(anchor.yaw)) +
                     (py - anchor.y) * std::cos(anchor.yaw);
  double dpsi = std::fmod(yaw - anchor.yaw + kPi, 2.0 * kPi);
  if (dpsi < 0.0) {
    dpsi += 2.0 * kPi;
  }
  dpsi -= kPi;
  err << e_y, dpsi, v, omega;
}

}  // namespace linear_mpc_controller
