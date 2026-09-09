// A4.2 SEEKING / PROBATION output shaping, extracted from the node's early
// return branch so the probation speed cap and the decel-to-zero behaviour
// are unit-testable without a live Nav2 controller node.
//
// Policy (verbatim mirror of the node branch it replaces):
//   * SEEKING cycles carry a zero command (the core returns v_cmd==0); the
//     external speed limit still applies but never raises it.
//   * PROBATION cycles are additionally capped to v_probation (the A4.2
//     observation window drives at reduced speed).
//   * omega is clamped to omega_max.
//   * the terminal stop (sqrt(2 a_max remaining)) still applies on the
//     ACCEPTED arc, so the shaped command never exceeds what is needed to
//     stop before path_len - terminal_stop_margin.
#ifndef LINEAR_MPC_CONTROLLER__SAFETY__REACQUIRE_COMMAND_HPP_
#define LINEAR_MPC_CONTROLLER__SAFETY__REACQUIRE_COMMAND_HPP_

#include <algorithm>
#include <cmath>

#include "linear_mpc_controller/mpc/linear_mpc.hpp"

namespace linear_mpc_controller
{

/// Parameters the node passes into shapeReacquireCommand (same fields it
/// used inline: params_ + speed_limit_* + terminal_stop_margin_).
struct ReacquireCommandParams
{
  double v_max = 1.5;
  double omega_max = 2.0;
  double a_max = 1.0;
  double v_probation = 0.15;
  double terminal_stop_margin = 0.20;
  bool speed_limit_active = false;
  double speed_limit = 0.0;
  bool speed_limit_percentage = false;
};

/// Shape the output command for one A4.2 SEEKING / PROBATION cycle.
inline void shapeReacquireCommand(
  const MpcCycleResult & res,
  const ReacquireCommandParams & p,
  double path_len,
  double & v_out,
  double & w_out)
{
  double v_allow = p.v_max;
  if (p.speed_limit_active) {
    const double lim = p.speed_limit_percentage
                         ? p.v_max * std::abs(p.speed_limit) / 100.0
                         : std::abs(p.speed_limit);
    v_allow = std::min(v_allow, lim);
  }
  if (res.in_probation) {
    v_allow = std::min(v_allow, p.v_probation);
  }
  const double v = std::clamp(res.v_cmd, -v_allow, v_allow);
  const double w = std::clamp(res.omega_cmd, -p.omega_max, p.omega_max);
  // terminal stop must still apply on the accepted arc
  const double remaining =
    std::max(0.0, path_len - res.accepted_arc - p.terminal_stop_margin);
  const double v_term = std::sqrt(2.0 * std::max(p.a_max, 1e-6) * remaining);
  v_out = std::clamp(v, -v_term, v_term);
  w_out = w;
}

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__SAFETY__REACQUIRE_COMMAND_HPP_
