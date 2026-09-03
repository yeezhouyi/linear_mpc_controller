#include "linear_mpc_controller/mpc/fallback_policy.hpp"

#include <algorithm>
#include <cmath>

namespace linear_mpc_controller
{

void FallbackPolicy::reset()
{
  active_ = false;
  degrade_cycles_ = 0;
  last_v_ = 0.0;
}

void FallbackPolicy::apply(double v, double omega, HealthState health,
  double & v_out, double & omega_out, HealthState & health_out, int & stage_out)
{
  v_out = 0.0;
  omega_out = 0.0;
  health_out = health;
  stage_out = 0;

  // 1) NaN / non-finite -> never emit garbage.
  if (!std::isfinite(v) || !std::isfinite(omega)) {
    health_out = HealthState::NAN_OUTPUT;
    stage_out = 3;
    active_ = false;
    degrade_cycles_ = 0;
    return;
  }

  // 2) Critical input conditions -> immediate safe zero.
  if (isCritical(health)) {
    health_out = health;
    stage_out = 3;
    active_ = false;
    degrade_cycles_ = 0;
    return;
  }

  // 3) QP failure -> deterministic geometric degrade-to-stop.
  if (isQpDegrade(health)) {
    if (!active_) {
      active_ = true;
      degrade_cycles_ = 0;
      last_v_ = std::max(std::fabs(v), 1e-6);
    }
    ++degrade_cycles_;
    const double factor =
      std::max(0.0, 1.0 - params_.stop_deceleration * params_.Ts / last_v_);
    last_v_ = last_v_ * factor;
    if (degrade_cycles_ > params_.max_hold_cycles) {
      health_out = HealthState::EMERGENCY_STOP;
      stage_out = 3;
      active_ = false;
      v_out = 0.0;
      omega_out = 0.0;
      return;
    }
    health_out = HealthState::FALLBACK_ACTIVE;
    stage_out = 2;
    v_out = std::max(last_v_, 0.0);
    omega_out = 0.0;  // heading hold while stopping
    return;
  }

  // 4) Healthy: clamp to hard actuator bounds.
  active_ = false;
  v_out = std::min(std::max(v, params_.v_min), params_.v_max);
  omega_out = std::min(std::max(omega, -params_.omega_max), params_.omega_max);
  const bool clamped = std::fabs(v_out - v) > 1e-12 || std::fabs(omega_out - omega) > 1e-12;
  if (clamped) {
    health_out = (health == HealthState::OK) ? HealthState::SAFETY_CLAMPED : health;
    stage_out = 1;
  }
}

}  // namespace linear_mpc_controller
