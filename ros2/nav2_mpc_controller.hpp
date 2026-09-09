// B6B: Nav2 controller_server plugin for the linear MPC core.
//
// SCOPE (read before touching): this is a TRAJECTORY TRACKER.  It does NOT
// read costmap obstacles and does NOT do collision checking -- avoidance is
// the global planner's and the downstream collision_monitor's job.  The
// costmap handle is only used to obtain the base frame id.  Do not turn
// "nav2 controller" into "obstacle-aware controller" here.
//
// Coordinate-frame discipline (silent-error trap #1): the reference and the
// A5.1/A5.2 projection gate carry arc state ACROSS cycles, and
// LinearMpcController::setReference() RESETS that gate.  Re-adapting /
// re-setting the plan every cycle would silently clear the gate every Ts --
// the whole A5 acceptance chain would degrade to a stateless argmin without
// a single error.  Therefore the ROBOT POSE is transformed INTO the path
// frame each cycle (one tf lookup); the reference and the gate never move.
//
// Failure contract (B6B.2): computeVelocityCommands either returns an
// executable velocity or throws a nav2_core ControllerException.  Silently
// returning zero velocity is the worst choice (the BT keeps RUNNING with a
// stopped robot).
//
// Terminal yaw semantics: a differential tracker converges to the path and
// therefore ARRIVES at the final pose with the path-end TANGENT heading --
// it cannot rotate in place to satisfy an arbitrary goal yaw.  A yaw-
// tolerant goal checker therefore needs a path whose final pose carries the
// tangential attitude (real Nav2 planners emit this).  A path whose final
// quaternion is identity on a curved segment makes the goal unsatisfiable
// in yaw; that property is what the terminal-stop negative control relies
// on (only the terminal clamp then stops the robot from driving out).
#ifndef LINEAR_MPC_CONTROLLER__ROS2__NAV2_MPC_CONTROLLER_HPP_
#define LINEAR_MPC_CONTROLLER__ROS2__NAV2_MPC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "linear_mpc_controller/mpc/linear_mpc.hpp"
#include "linear_mpc_controller/safety/qp_fail_monitor.hpp"
#include "linear_mpc_controller/safety/reacquire_command.hpp"

namespace linear_mpc_controller
{

class LinearMpcNav2Controller : public nav2_core::Controller
{
public:
  LinearMpcNav2Controller() = default;
  ~LinearMpcNav2Controller() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & plan) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit,
                     const bool & percentage) override;

protected:
  // shared tf buffer + node plumbing
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_mpc_controller")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // frames
  std::string base_frame_id_;     // from costmap
  std::string odom_frame_id_ = "odom";
  std::string path_frame_id_ = "map";

  // MpcParams consumed by the core (set at configure)
  MpcParams params_;

  // nav2 external speed limit (B6B.1: independent of probation)
  bool speed_limit_active_ = false;
  double speed_limit_ = 0.0;
  bool speed_limit_percentage_ = false;

  // core
  std::unique_ptr<LinearMpcController> controller_;
  bool has_plan_ = false;
  double path_len_ = 0.0;

  // terminal-stop margin (recorded: without it the tracker ran 13.37 m past
  // a 2.36 m arc because sampleByArc clamps at the last point)
  double terminal_stop_margin_ = 0.20;

  // B6B.2 counters
  // QP-failure accounting lives in safety/qp_fail_monitor.hpp so the abort
  // policy is unit-testable; max is synced from qp_fail_max_ at configure.
  QpFailMonitor qp_fail_monitor_;
  int qp_fail_max_ = 3;
  int ambiguous_run_ = 0;
  int ambiguous_max_ = 5;

  double tf_tolerance_ = 0.2;
  long long cycle_diag_ = 0;
  double last_cmd_v_ = 0.0;
  double last_cmd_w_ = 0.0;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__ROS2__NAV2_MPC_CONTROLLER_HPP_
