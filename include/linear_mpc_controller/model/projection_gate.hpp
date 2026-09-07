// A5B.1 C++ mirror of mpc_core/progress_gate.py (ProgressAllowanceGate).
// One physical-displacement reachability gate per ledger/controller channel.
// Constants are numerically identical to the Python module and to MpcParams
// (see test_gate_constants_match_the_controller); changing one side without
// the other reintroduces the class of bug this gate exists to remove.
#ifndef LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_GATE_HPP_
#define LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_GATE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace linear_mpc_controller
{

// A5.1 constants (numeric copies of mpc_core.progress_gate / MpcParams).
inline constexpr double kProjectionMarginM = 0.02;    // noise floor per cycle
inline constexpr double kAllowanceCapM = 0.30;        // max banked budget
inline constexpr double kOdomStepNoiseM = 0.015;      // per-step odom noise

/// Outcome of one gate cycle; all arc quantities are metres.
struct GateDecision
{
  bool accepted = false;
  double delta_s = 0.0;        // raw_arc - baseline BEFORE this cycle
  double limit = 0.0;          // margin + banked budget used for the decision
  double physical_ds = 0.0;    // displacement banked this cycle (capped)
  double budget = 0.0;         // budget AFTER the decision (0 on accept)
  double prev_accepted_arc = 0.0;
  double accepted_arc = 0.0;
};

/// Physical-displacement reachability gate (A5.1 arithmetic), C++ side.
/// Mirrors ProgressAllowanceGate.step(): the allowance is banked PHYSICAL
/// displacement projected on the tangent at the last ACCEPTED arc, capped at
/// v_max*Ts + odom noise; the budget is banked BEFORE the decision and spent
/// on accept; the accepted baseline moves FORWARD ONLY.
struct ProjectionGateState
{
  double v_max = 1.5;
  double Ts = 0.05;
  double margin_m = kProjectionMarginM;
  double cap_m = kAllowanceCapM;
  double odom_step_noise_m = kOdomStepNoiseM;
  double accepted_arc = 0.0;
  double budget = 0.0;
  bool has_last_pose = false;
  double last_x = 0.0;
  double last_y = 0.0;

  ProjectionGateState() = default;
  ProjectionGateState(double v_max_in, double Ts_in,
                      double accepted_arc_in = 0.0,
                      double margin_m_in = kProjectionMarginM,
                      double cap_m_in = kAllowanceCapM,
                      double odom_step_noise_m_in = kOdomStepNoiseM)
  : v_max(v_max_in), Ts(Ts_in), margin_m(margin_m_in), cap_m(cap_m_in),
    odom_step_noise_m(odom_step_noise_m_in), accepted_arc(accepted_arc_in)
  {
  }

  double stepCap() const
  {
    return v_max * Ts + odom_step_noise_m;
  }

  /// One gate cycle.  tangent_yaw_at_baseline is the path tangent at the last
  /// ACCEPTED arc (never the raw candidate's segment).  x/y are the current
  /// pose; raw_arc the raw projection arc of this pose.
  GateDecision step(double x, double y, double raw_arc,
                    double tangent_yaw_at_baseline)
  {
    GateDecision d;
    d.prev_accepted_arc = accepted_arc;
    // 1) allowance from real motion along the path tangent (A5.1).
    if (!has_last_pose) {
      d.physical_ds = 0.0;
    } else {
      const double dx = x - last_x;
      const double dy = y - last_y;
      const double projected = dx * std::cos(tangent_yaw_at_baseline) +
                               dy * std::sin(tangent_yaw_at_baseline);
      d.physical_ds = std::min(std::max(projected, 0.0), stepCap());
    }
    // Banked BEFORE the decision (otherwise delta_s stays a cycle ahead
    // whenever v*Ts > margin -- the STALL failure mode).
    budget = std::min(budget + d.physical_ds, cap_m);
    has_last_pose = true;
    last_x = x;
    last_y = y;
    // 2) decision: baseline is the ACCEPTED arc, never the last raw one.
    d.delta_s = raw_arc - d.prev_accepted_arc;
    d.limit = margin_m + budget;
    d.accepted = d.delta_s <= d.limit;
    if (d.accepted) {
      budget = 0.0;
      if (raw_arc > accepted_arc) {
        accepted_arc = raw_arc;   // forward only
      }
    }
    d.budget = budget;
    d.accepted_arc = accepted_arc;
    return d;
  }

  /// Capture-resolve re-anchor (A5.2), FORWARD ONLY; spends the budget.
  void rebaseline(double arc)
  {
    if (arc > accepted_arc) {
      accepted_arc = arc;
    }
    budget = 0.0;
  }

  void reset(double accepted_arc_in = 0.0)
  {
    accepted_arc = accepted_arc_in;
    budget = 0.0;
    has_last_pose = false;
    last_x = last_y = 0.0;
  }
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MODEL__PROJECTION_GATE_HPP_
