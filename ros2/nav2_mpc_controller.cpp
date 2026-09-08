#include "nav2_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "trajectory_adapter.hpp"

namespace linear_mpc_controller
{

namespace
{
double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}
}  // namespace

void LinearMpcNav2Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  auto node = parent.lock();
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring %s", plugin_name_.c_str());

  // costmap handle is used ONLY for the base frame id (scope comment).
  if (costmap_ros != nullptr) {
    base_frame_id_ = costmap_ros->getBaseFrameID();
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_frame_id", rclcpp::ParameterValue("odom"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".Ts", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".N", rclcpp::ParameterValue(25));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".v_max", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".v_min", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".omega_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".a_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".qp_max_iter", rclcpp::ParameterValue(1500));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_stop_margin", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".qp_fail_max", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".ambiguous_max", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tf_tolerance", rclcpp::ParameterValue(0.2));

  odom_frame_id_ = node->get_parameter(name + ".odom_frame_id").as_string();
  params_.Ts = node->get_parameter(name + ".Ts").as_double();
  params_.N = node->get_parameter(name + ".N").as_int();
  params_.v_max = node->get_parameter(name + ".v_max").as_double();
  params_.v_min = node->get_parameter(name + ".v_min").as_double();
  params_.omega_max = node->get_parameter(name + ".omega_max").as_double();
  params_.a_max = node->get_parameter(name + ".a_max").as_double();
  params_.qp_max_iter = node->get_parameter(name + ".qp_max_iter").as_int();
  terminal_stop_margin_ =
    node->get_parameter(name + ".terminal_stop_margin").as_double();
  qp_fail_max_ = node->get_parameter(name + ".qp_fail_max").as_int();
  ambiguous_max_ = node->get_parameter(name + ".ambiguous_max").as_int();
  tf_tolerance_ = node->get_parameter(name + ".tf_tolerance").as_double();

  RCLCPP_INFO(
    logger_,
    "%s configured: Ts=%.3f N=%d v_max=%.2f omega_max=%.2f "
    "terminal_stop_margin=%.2f",
    plugin_name_.c_str(), params_.Ts, params_.N, params_.v_max,
    params_.omega_max, terminal_stop_margin_);
}

void LinearMpcNav2Controller::cleanup() { controller_.reset(); }

void LinearMpcNav2Controller::activate()
{
  RCLCPP_INFO(logger_, "%s activated", plugin_name_.c_str());
}

void LinearMpcNav2Controller::deactivate()
{
  RCLCPP_INFO(logger_, "%s deactivated", plugin_name_.c_str());
}

void LinearMpcNav2Controller::setPlan(const nav_msgs::msg::Path & plan)
{
  if (plan.poses.size() < 2) {
    // B6B.2: fail HERE, never accept and fail later in
    // computeVelocityCommands -- the error location must match its cause.
    throw nav2_core::InvalidPath("B6B: plan has < 2 poses");
  }
  path_frame_id_ = plan.header.frame_id;
  AdapterParams ap;
  ap.v_default = std::min(params_.v_max, 0.5);
  ap.v_max = params_.v_max;
  std::vector<TrackPoint> traj = adaptPath(plan, ap);
  if (traj.size() < 2) {
    throw nav2_core::InvalidPath("B6B: adaptPath produced < 2 points");
  }
  controller_ = std::make_unique<LinearMpcController>(params_, traj);
  controller_->setReference(traj);   // resets gate + reacquire (A5.1/A4.2)
  path_len_ = traj.back().s;
  has_plan_ = true;
  qp_fail_run_ = 0;
  ambiguous_run_ = 0;
  last_cmd_v_ = last_cmd_w_ = 0.0;
  RCLCPP_INFO(logger_, "setPlan: %zu points, L=%.3f m in frame %s",
              traj.size(), path_len_, path_frame_id_.c_str());
}

geometry_msgs::msg::TwistStamped LinearMpcNav2Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  if (!has_plan_ || !controller_) {
    throw nav2_core::NoValidControl("B6B: no plan set");
  }
  // Frame discipline: transform the ROBOT POSE into the PATH frame (one tf
  // lookup per cycle).  NEVER re-adapt/re-setPlan per cycle -- that resets
  // the projection gate every Ts (silent-error trap #1).
  geometry_msgs::msg::PoseStamped pose_path;
  if (pose.header.frame_id != path_frame_id_) {
    try {
      pose_path = tf_->transform(
        pose, path_frame_id_,
        tf2::durationFromSec(tf_tolerance_));
    } catch (const tf2::TransformException & e) {
      // B6B.2: untrustworthy pose -> no extrapolation with stale TF
      throw nav2_core::ControllerTFError("B6B: tf transform failed: " +
                                         std::string(e.what()));
    }
  } else {
    pose_path = pose;
  }
  const double px = pose_path.pose.position.x;
  const double py = pose_path.pose.position.y;
  const double yaw = wrapAngle(2.0 * std::atan2(
    pose_path.pose.orientation.z, pose_path.pose.orientation.w));
  const double v_in = velocity.linear.x;
  const double w_in = velocity.angular.z;

  const MpcCycleResult res = controller_->computeCycle(px, py, yaw, v_in, w_in);

  // ---- B6B.2 failure contract ---------------------------------------
  if (res.health == HealthState::EMERGENCY_STOP) {
    if (res.reason.find("PROJECTION_LOST") != std::string::npos) {
      // A4.2 seeking timed out: the ONLY exit of the reacquire protocol on
      // the Nav2 side.  Reset the gate state so a later setPlan starts clean.
      has_plan_ = false;
      controller_.reset();
      throw nav2_core::NoValidControl("B6B: PROJECTION_LOST");
    }
    if (res.reason.find("PROJECTION_AMBIGUOUS") != std::string::npos ||
        res.projection_stage == 3) {
      ++ambiguous_run_;
      if (ambiguous_run_ > ambiguous_max_) {
        has_plan_ = false;
        controller_.reset();
        throw nav2_core::NoValidControl("B6B: PROJECTION_AMBIGUOUS");
      }
      geometry_msgs::msg::TwistStamped stop;
      stop.header.stamp = pose.header.stamp;
      stop.twist.linear.x = 0.0;
      stop.twist.angular.z = 0.0;
      return stop;
    }
  }
  if (res.qp_status == QpSolution::Status::kFailed) {
    ++qp_fail_run_;
    if (qp_fail_run_ > qp_fail_max_) {
      has_plan_ = false;
      controller_.reset();
      throw nav2_core::NoValidControl("B6B: QP failed beyond qp_fail_max");
    }
  } else if (qp_fail_run_ > 0 && res.qp_status == QpSolution::Status::kSolved) {
    --qp_fail_run_;   // recovery credit
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = base_frame_id_;

  if (res.reacquire_seeking || res.in_probation) {
    // A4.2: bounded decel-to-zero / probation -- return zero-or-capped,
    // NEVER throw (bounded by reacquire_timeout_steps / probation_steps).
    double v_allow = params_.v_max;
    if (speed_limit_active_) {
      const double lim = speed_limit_percentage_
                           ? params_.v_max * std::abs(speed_limit_) / 100.0
                           : std::abs(speed_limit_);
      v_allow = std::min(v_allow, lim);
    }
    if (res.in_probation) {
      v_allow = std::min(v_allow, params_.v_probation);
    }
    const double v = std::clamp(res.v_cmd, -v_allow, v_allow);
    const double w = std::clamp(
      res.omega_cmd, -params_.omega_max, params_.omega_max);
    // terminal stop must still apply on the accepted arc
    const double remaining =
      std::max(0.0, path_len_ - res.accepted_arc - terminal_stop_margin_);
    const double v_term =
      std::sqrt(2.0 * std::max(params_.a_max, 1e-6) * remaining);
    cmd.twist.linear.x = std::clamp(v, -v_term, v_term);
    cmd.twist.angular.z = w;
    last_cmd_v_ = cmd.twist.linear.x;
    last_cmd_w_ = cmd.twist.angular.z;
    return cmd;
  }

  // ---- B6B.1: two independent speed limits, take the min -------------
  // external setSpeedLimit() writes speed_limit_* ONLY; A4.2 probation
  // writes nothing here (it is applied inside the core via v_probation).
  double v_allow = params_.v_max;
  if (speed_limit_active_) {
    const double lim = speed_limit_percentage_
                         ? params_.v_max * std::abs(speed_limit_) / 100.0
                         : std::abs(speed_limit_);
    v_allow = std::min(v_allow, lim);
  }
  double v = std::clamp(res.v_cmd, -v_allow, v_allow);
  const double w = std::clamp(
    res.omega_cmd, -params_.omega_max, params_.omega_max);

  // ---- terminal stop (real controller feature, docx B6B spec part 3) --
  // sampleByArc clamps at the last trajectory point, so past the end the
  // reference is a fixed point with NONZERO reference speed and the tracker
  // would chase it forever.  Decelerate on the ACCEPTED arc (A5.2), never
  // the raw one.  terminal_stop_margin is a parameter; the docx recorded a
  // real 13.37 m overshoot on a 2.36 m arc when this was missing.
  const double remaining =
    std::max(0.0, path_len_ - res.accepted_arc - terminal_stop_margin_);
  const double v_term =
    std::sqrt(2.0 * std::max(params_.a_max, 1e-6) * remaining);
  v = std::clamp(v, -v_term, v_term);

  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;
  last_cmd_v_ = v;
  last_cmd_w_ = w;
  return cmd;
}

void LinearMpcNav2Controller::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  // B6B.1: this method NEVER touches probation state; it only records the
  // external (Speed Filter / BT / manual) limit.  The composite min is
  // applied at output time.
  if (speed_limit < 0.0) {
    speed_limit_active_ = false;
    return;
  }
  speed_limit_active_ = true;
  speed_limit_percentage_ = percentage;
  speed_limit_ = speed_limit;
}

}  // namespace linear_mpc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  linear_mpc_controller::LinearMpcNav2Controller, nav2_core::Controller)
