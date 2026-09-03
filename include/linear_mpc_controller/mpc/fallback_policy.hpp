// Fallback & health contract (U4). See mpc_core/fallback.py for the verified
// reference behaviour this mirrors.
#ifndef LINEAR_MPC_CONTROLLER__MPC__FALLBACK_POLICY_HPP_
#define LINEAR_MPC_CONTROLLER__MPC__FALLBACK_POLICY_HPP_

namespace linear_mpc_controller
{

enum class HealthState : int
{
  OK = 0,
  NO_REFERENCE = 1,
  STALE_REFERENCE = 2,
  TF_INVALID = 3,
  STATE_STALE = 4,
  QP_TIMEOUT = 5,
  QP_INFEASIBLE = 6,
  NAN_OUTPUT = 7,
  SAFETY_CLAMPED = 8,
  FALLBACK_ACTIVE = 9,
  EMERGENCY_STOP = 10,
};

/// Instant-stop set: never reuse the last non-zero command after these.
inline bool isCritical(HealthState h)
{
  switch (h) {
    case HealthState::NO_REFERENCE:
    case HealthState::STALE_REFERENCE:
    case HealthState::TF_INVALID:
    case HealthState::STATE_STALE:
    case HealthState::NAN_OUTPUT:
    case HealthState::EMERGENCY_STOP:
      return true;
    default:
      return false;
  }
}

/// Degrade-to-stop set: deterministic geometric deceleration on QP failure.
inline bool isQpDegrade(HealthState h)
{
  return h == HealthState::QP_TIMEOUT || h == HealthState::QP_INFEASIBLE ||
         h == HealthState::FALLBACK_ACTIVE;
}

struct FallbackParams
{
  double Ts = 0.05;
  double v_min = 0.0;
  double v_max = 1.5;
  double omega_max = 2.0;
  double stop_deceleration = 0.8;  // m/s^2 used by the degrade ladder
  int max_hold_cycles = 10;        // before EMERGENCY_STOP
};

/// Deterministic fallback state machine.  Not thread safe (controller-owned).
class FallbackPolicy
{
public:
  explicit FallbackPolicy(const FallbackParams & p) : params_(p) {}

  void reset();

  /// Apply the ladder; returns (v_cmd, omega_cmd, health, stage).
  /// Stage: 0 ok, 1 clamped, 2 degrade-to-stop, 3 emergency/zero.
  void apply(double v, double omega, HealthState health,
    double & v_out, double & omega_out, HealthState & health_out, int & stage_out);

private:
  FallbackParams params_;
  bool active_ = false;
  int degrade_cycles_ = 0;
  double last_v_ = 0.0;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MPC__FALLBACK_POLICY_HPP_
