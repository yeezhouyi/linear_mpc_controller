// velocity_arbiter: single writer of /cmd_vel (KTD4 / R8).
// Subscribes to the controller candidates (cmd_vel_mpc / cmd_vel_nav /
// cmd_vel_stop) and republishes exactly one selected twist on /cmd_vel.
// The selection policy is deterministic: any active stop request wins;
// otherwise the highest-priority controller with fresh output.  Timeout of a
// candidate source (no message within its timeout) demotes it.
//
// NOTE: compile & integration gate = WSL2 colcon (see README).
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace linear_mpc_controller
{

class VelocityArbiter : public rclcpp::Node
{
public:
  VelocityArbiter() : rclcpp::Node("velocity_arbiter")
  {
    declare_parameter("source_timeout_s", 0.5);
    source_timeout_s_ = get_parameter("source_timeout_s").as_double();
    out_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS());

    // Highest priority: stop; then mpc; then nav (future).  Every source
    // publishes TwistStamped so we can enforce freshness with stamps.
    mk_source("cmd_vel_stop", 3);
    mk_source("cmd_vel_mpc", 2);
    mk_source("cmd_vel_nav", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
      std::bind(&VelocityArbiter::tick, this));
  }

private:
  struct Source
  {
    std::string topic;
    int priority;                 // higher wins
    geometry_msgs::msg::TwistStamped last;
    bool have = false;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
  };

  void mk_source(const std::string & topic, int priority)
  {
    Source s;
    s.topic = topic;
    s.priority = priority;
    s.sub = create_subscription<geometry_msgs::msg::TwistStamped>(
      topic, rclcpp::SystemDefaultsQoS(),
      [this, topic](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        for (auto & src : sources_) {
          if (src.topic == topic) { src.last = *msg; src.have = true; return; }
        }
      });
    sources_.push_back(std::move(s));
  }

  void tick()
  {
    geometry_msgs::msg::TwistStamped out;
    int best = -1;
    const auto now = this->now();
    for (auto & s : sources_) {
      if (!s.have) continue;
      if ((now - s.last.header.stamp).seconds() > source_timeout_s_) {
        s.have = false;  // demote stale source
        continue;
      }
      if (s.priority > best) { best = s.priority; out = s.last; }
    }
    // no fresh source -> publish zero velocity (fail-safe)
    if (best < 0) {
      out.header.stamp = now;
      out.twist.linear.x = 0.0;
      out.twist.angular.z = 0.0;
    }
    out.header.frame_id = "base_link";
    out_pub_->publish(out);
  }

  double source_timeout_s_ = 0.5;
  std::vector<Source> sources_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr out_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace linear_mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<linear_mpc_controller::VelocityArbiter>());
  rclcpp::shutdown();
  return 0;
}
