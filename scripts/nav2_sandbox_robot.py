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
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SandboxRobot(Node):
    def __init__(self):
        super().__init__("nav2_sandbox_robot")
        self.declare_parameter("publish_rate", 50.0)
        rate = self.get_parameter("publish_rate").value
        self.period = 1.0 / rate

        # B6B stage 2b: the arc scenario must START OFFLINE (offset pose)
        # so the steering law is excited; otherwise max|omega| never rises.
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)

        self.lock = threading.Lock()
        self.v = 0.0
        self.w = 0.0
        self.v_ts_t = 0.0
        self.w_ts_t = 0.0
        self.robot_x = self.get_parameter("start_x").value
        self.robot_y = self.get_parameter("start_y").value
        self.robot_yaw = self.get_parameter("start_yaw").value
        self.now = self.get_clock().now()

        # Subscribe the cmd_vel flavour that actually exists -- publishing
        # Twist AND TwistStamped on one topic makes the RMW complain about
        # type mismatches and pollutes debugging.  Probe the topic type
        # unless the caller pins it explicitly with cmd_vel_type:=.
        self.declare_parameter("cmd_vel_type", "")   # "" = auto-probe
        ctype = self.get_parameter("cmd_vel_type").value
        if not ctype:
            ctype = self._probe_type()

        if ctype == "TwistStamped":
            self.sub = self.create_subscription(
                TwistStamped, "/cmd_vel", self.cb_ts, 10)
            self.get_logger().info("subscribing /cmd_vel TwistStamped")
        else:
            self.sub = self.create_subscription(
                Twist, "/cmd_vel", self.cb_tw, 10)
            self.get_logger().info("subscribing /cmd_vel Twist")

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = TransformBroadcaster(self)

        # Empty occupancy grid (latched) so the controller_server's costmap
        # static layer receives a map and the costmap becomes "current" --
        # otherwise controller_server aborts with "Costmap timed out waiting
        # for update".
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.info.resolution = 0.1
        m.info.width = 60
        m.info.height = 60
        m.info.origin.position.x = -3.0
        m.info.origin.position.y = -3.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * (60 * 60)
        self.map_pub.publish(m)

        self.get_logger().info("sandbox robot up (rate %.0f Hz)" % rate)
        self.timer = self.create_timer(self.period, self.step)

    def _probe_type(self):
        import subprocess
        try:
            out = subprocess.run(
                ["ros2", "topic", "info", "/cmd_vel", "-t"],
                capture_output=True, text=True, timeout=10).stdout
            for ln in out.splitlines():
                if "Type:" in ln:
                    return ln.split(":")[-1].strip().split("/")[-1]
        except Exception:
            pass
        # Jazzy controller_server default is TwistStamped
        return "TwistStamped"

    def cb_ts(self, msg: TwistStamped):
        with self.lock:
            self.v = msg.twist.linear.x
            self.w = msg.twist.angular.z
            self.v_ts_t = time.monotonic()
        if not hasattr(self, "_got"):
            self._got = True
            self.get_logger().info(
                "FIRST cmd_vel TwistStamped: v=%.3f w=%.3f" % (self.v, self.w))

    def cb_tw(self, msg: Twist):
        with self.lock:
            self.v = msg.linear.x
            self.w = msg.angular.z
            self.w_ts_t = time.monotonic()
        if not hasattr(self, "_got"):
            self._got = True
            self.get_logger().info(
                "FIRST cmd_vel Twist: v=%.3f w=%.3f" % (self.v, self.w))

    def step(self):
        with self.lock:
            v, w = self.v, self.w
        dt = self.period
        self.robot_x += v * math.cos(self.robot_yaw) * dt
        self.robot_y += v * math.sin(self.robot_yaw) * dt
        self.robot_yaw += w * dt
        self.now = self.get_clock().now()
        t = self.now.to_msg()

        # Static identity map->odom so the controller_server's in-process
        # costmap (default global_frame=map) resolves base_link->map.
        st = TransformStamped()
        st.header = Header(stamp=t, frame_id="map")
        st.child_frame_id = "odom"
        st.transform.rotation.w = 1.0
        self.static_br.sendTransform(st)

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
