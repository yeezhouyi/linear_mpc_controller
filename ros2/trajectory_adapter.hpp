// Trajectory adapter: nav_msgs/Path (poses only) -> dense TrackPoint vector.
// Deterministic completion rule (docs/ros2_interface_contract.md §3):
// tangent from neighbour chords, curvature by heading difference, speed by
// curvature cap with a default; arc length by cumulative chord.
#ifndef LINEAR_MPC_CONTROLLER__ROS2__TRAJECTORY_ADAPTER_HPP_
#define LINEAR_MPC_CONTROLLER__ROS2__TRAJECTORY_ADAPTER_HPP_

#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "nav_msgs/msg/path.hpp"

namespace linear_mpc_controller
{

struct AdapterParams
{
  double v_default = 0.5;   // m/s
  double v_max = 1.5;       // m/s
  double curve_speed = 0.6; // m/s reference inside the curvature cap
  // Kinematic angular-velocity bound for the completed reference.  MUST stay
  // equal to MpcParams::omega_max (mpc/qp_problem.hpp, currently 2.0 rad/s):
  // the completion rule v <= omega_max/|kappa| is what makes the reference
  // trackable, and the same constant guards the QP input.
  double omega_max = 2.0;   // rad/s
};

/// Convert a Path into TrackPoints, completing yaw/kappa/v/s deterministically.
/// Empty or single-point paths yield an empty vector.
/// The speed completion enforces |kappa| * v <= omega_max (no floor after the
/// cap: a floor would push |kappa|*v back over the bound).
std::vector<TrackPoint> adaptPath(const nav_msgs::msg::Path & path, const AdapterParams & p);

/// Largest |kappa| * v / omega_max over the points (0.0 when empty).
/// Mirror of mpc_core trajectory_tools.omega_violations' max_ratio: a
/// reference entering a tracker must satisfy max_ratio <= 1 + 1e-9.
double maxOmegaRatio(const std::vector<TrackPoint> & pts, double omega_max);

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__ROS2__TRAJECTORY_ADAPTER_HPP_
