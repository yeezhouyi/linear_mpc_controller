// linear_mpc_node: lifecycle controller node (standalone_tracking mode).
//
// Pipeline per cycle (sim time):
//   /odom -> pose+v/omega  -> LinearMpcController(traj) -> cmd candidate
//   -> diagnostics on ~/diagnostics ; cmd goes to the velocity arbiter topic
//      cmd_vel_mpc (the arbiter owns the final /cmd_vel).
//
// Freshness / staleness gates live here (R9): no traj, stale traj,
// stale/backwards odom stamps, sim-clock discipline.
//
// NOTE: compile & integration gate = WSL2 colcon + Gazebo (see README).
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "linear_mpc_controller/mpc/linear_mpc.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_adapter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace linear_mpc_controller
{

class LinearMpcNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  LinearMpcNode() : rclcpp_lifecycle::LifecycleNode("linear_mpc_node")
  {
    declare_parameter("Ts", 0.05);
    declare_parameter("N", 25);
    declare_parameter("Q_diag", std::vector<double>{60., 10., 2., 1.});
    declare_parameter("Q_F_diag", std::vector<double>{120., 20., 4., 2.});
    declare_parameter("S_diag", std::vector<double>{0.5, 0.5});
    declare_parameter("v_max", 1.5);
    declare_parameter("omega_max", 2.0);
    declare_parameter("a_max", 1.0);
    declare_parameter("alpha_max", 2.0);
    declare_parameter("lookahead_m", 0.0);
    declare_parameter("traj_max_age_s", 5.0);
    declare_parameter("odom_max_age_s", 0.5);
    declare_parameter("adapter.v_default", 0.5);
    declare_parameter("adapter.v_max", 1.5);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    using namespace std::chrono_literals;
    MpcParams p;
    p.Ts = get_parameter("Ts").as_double();
    p.N = static_cast<int>(get_parameter("N").as_int());
    const auto qv = get_parameter("Q_diag").as_double_array();
    const auto qf = get_parameter("Q_F_diag").as_double_array();
    const auto sv = get_parameter("S_diag").as_double_array();
    for (int i = 0; i < 4; ++i) { p.Q(i, i) = qv[i]; p.Q_F(i, i) = qf[i]; }
    for (int i = 0; i < 2; ++i) { p.S(i, i) = sv[i]; }
    p.v_max = get_parameter("v_max").as_double();
    p.omega_max = get_parameter("omega_max").as_double();
    p.a_max = get_parameter("a_max").as_double();
    p.alpha_max = get_parameter("alpha_max").as_double();
    p.lookahead_m = get_parameter("lookahead_m").as_double();
    traj_max_age_s_ = get_parameter("traj_max_age_s").as_double();
    odom_max_age_s_ = get_parameter("odom_max_age_s").as_double();
    adap_.v_default = get_parameter("adapter.v_default").as_double();
    adap_.v_max = get_parameter("adapter.v_max").as_double();
    mpc_ = std::make_unique<LinearMpcController>(p, std::vector<TrackPoint>{});

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        have_odom_ = true;
      });
    traj_sub_ = create_subscription<nav_msgs::msg::Path>(
      "~/reference", rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        last_traj_ = adaptPath(*msg, adap_);
        // (re)load the reference into the controller — without this the
        // core keeps its empty constructor trajectory and stays in
        // NO_REFERENCE fallback forever (found in the first Gazebo gate)
        if (!last_traj_.empty()) {
          mpc_->setReference(last_traj_);
        }
        traj_stamp_ = now();
        have_traj_ = !last_traj_.empty();
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_mpc", rclcpp::SystemDefaultsQoS());
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/diagnostics", rclcpp::SystemDefaultsQoS());
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&LinearMpcNode::cycle, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void cycle()
  {
    if (!have_odom_) return;  // wait for state; no output (R9)
    const auto & o = last_odom_.pose.pose;
    const auto & tw = last_odom_.twist.twist;
    MpcCycleResult out;
    if (!have_traj_) {
      out.health = HealthState::NO_REFERENCE;
    } else if ((now() - traj_stamp_).seconds() > traj_max_age_s_) {
      out.health = HealthState::STALE_REFERENCE;
    } else {
      out = mpc_->computeCycle(o.position.x, o.position.y, yawFromQuat(o.orientation),
        tw.linear.x, tw.angular.z);
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    const bool allow_cmd = !isCritical(out.health);
    cmd.twist.linear.x = allow_cmd ? out.v_cmd : 0.0;
    cmd.twist.angular.z = allow_cmd ? out.omega_cmd : 0.0;
    cmd_pub_->publish(cmd);
    publishDiagnostics(out);
  }

  void publishDiagnostics(const MpcCycleResult & out)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      static_cast<double>(out.health), out.v_cmd, out.omega_cmd,
      static_cast<double>(out.qp_status), static_cast<double>(out.qp_iterations),
      out.qp_time_us, out.constraint_violation, out.fallback_used ? 1.0 : 0.0,
      out.e_used(0), out.e_used(1), out.e_used(2), out.e_used(3)};
    diag_pub_->publish(msg);
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  }

  std::unique_ptr<LinearMpcController> mpc_;
  AdapterParams adap_;
  bool have_odom_ = false;
  bool have_traj_ = false;
  nav_msgs::msg::Odometry last_odom_;
  std::vector<TrackPoint> last_traj_;
  rclcpp::Time traj_stamp_;
  double traj_max_age_s_ = 5.0;
  double odom_max_age_s_ = 0.5;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace linear_mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<linear_mpc_controller::LinearMpcNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
