#!/usr/bin/env python3
"""B6B sandbox fake robot: an ideal single-wheel integrator.

Publishes odom (nav_msgs/Odometry) + tf (odom->base_link) integrated from
the incoming velocity command, so a Nav2 controller_server + plugin can be
exercised without Gazebo.

Jazzy pitfall (B6B spec): the controller_server may emit TwistStamped OR
(un)Twist depending on configuration -- a fake robot that subscribes to the
wrong flavour silently never moves.  This node subscribes to BOTH and uses
whichever has data in the last 0.1 s.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SandboxRobot(Node):
    def __init__(self):
        super().__init__("nav2_sandbox_robot")
        self.declare_parameter("publish_rate", 50.0)
        rate = self.get_parameter("publish_rate").value
        self.period = 1.0 / rate

        self.lock = threading.Lock()
        self.v = 0.0
        self.w = 0.0
        self.v_ts_t = 0.0
        self.w_ts_t = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.now = self.get_clock().now()

        self.sub_ts = self.create_subscription(
            TwistStamped, "/cmd_vel", self.cb_ts, 10)
        self.sub_tw = self.create_subscription(
            Twist, "/cmd_vel", self.cb_tw, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br = TransformBroadcaster(self)

        self.get_logger().info("sandbox robot up (rate %.0f Hz)" % rate)
        self.timer = self.create_timer(self.period, self.step)

    def cb_ts(self, msg: TwistStamped):
        with self.lock:
            self.v = msg.twist.linear.x
            self.w = msg.twist.angular.z
            self.v_ts_t = time.monotonic()

    def cb_tw(self, msg: Twist):
        with self.lock:
            self.v = msg.linear.x
            self.w = msg.angular.z
            self.w_ts_t = time.monotonic()

    def step(self):
        with self.lock:
            v, w = self.v, self.w
        dt = self.period
        self.robot_x += v * math.cos(self.robot_yaw) * dt
        self.robot_y += v * math.sin(self.robot_yaw) * dt
        self.robot_yaw += w * dt
        self.now = self.get_clock().now()
        t = self.now.to_msg()

        odom = Odometry()
        odom.header = Header(stamp=t, frame_id="odom")
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header = Header(stamp=t, frame_id="odom")
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.robot_x
        tf_msg.transform.translation.y = self.robot_y
        tf_msg.transform.rotation.z = odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_br.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = SandboxRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
