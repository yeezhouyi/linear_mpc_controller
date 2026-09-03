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
};

/// Convert a Path into TrackPoints, completing yaw/kappa/v/s deterministically.
/// Empty or single-point paths yield an empty vector.
std::vector<TrackPoint> adaptPath(const nav_msgs::msg::Path & path, const AdapterParams & p);

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__ROS2__TRAJECTORY_ADAPTER_HPP_
