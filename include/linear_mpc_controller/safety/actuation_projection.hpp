// Runtime safety split (U10): actuation projection lives in C++ core; the
// collision gate consumes external costmap/monitor state (UNKNOWN -> stop).
// This MVP exposes only the actuation clamp; the collision gate is wired
// when a costmap interface exists (documented limitation, KTD12/R18).
#ifndef LINEAR_MPC_CONTROLLER__SAFETY__ACTUATION_PROJECTION_HPP_
#define LINEAR_MPC_CONTROLLER__SAFETY__ACTUATION_PROJECTION_HPP_

#include <algorithm>

namespace linear_mpc_controller
{

struct ActuationLimits
{
  double v_min = 0.0;
  double v_max = 1.5;
  double omega_max = 2.0;
};

/// Project (v, omega) onto the hard actuation box.  Returns whether clamped.
inline bool projectActuation(double & v, double & omega, const ActuationLimits & lim)
{
  const double v0 = v, w0 = omega;
  v = std::min(std::max(v, lim.v_min), lim.v_max);
  omega = std::min(std::max(omega, -lim.omega_max), lim.omega_max);
  return v != v0 || omega != w0;
}

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__SAFETY__ACTUATION_PROJECTION_HPP_
