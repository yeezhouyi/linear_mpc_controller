#!/usr/bin/env python3
"""A4 motion gate: PASS iff /odom start-end planar displacement > threshold."""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MotionCheck(Node):
    def __init__(self, topic, seconds, threshold):
        super().__init__("motion_check")
        self.first = None
        self.last = None
        self.start = time.monotonic()
        self.seconds = seconds
        self.threshold = threshold
        self.result = 2
        self.sub = self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.timer = self.create_timer(0.2, self.finish_if_ready)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        point = (float(p.x), float(p.y))
        if self.first is None:
            self.first = point
        self.last = point

    def finish_if_ready(self):
        if time.monotonic() - self.start < self.seconds:
            return
        if self.first is None or self.last is None:
            self.get_logger().error("FAIL: no odom samples")
        else:
            distance = math.hypot(
                self.last[0] - self.first[0],
                self.last[1] - self.first[1],
            )
            if distance > self.threshold:
                self.result = 0
                self.get_logger().info(
                    "PASS: displacement=%.3f m" % distance)
            else:
                self.get_logger().error(
                    "FAIL: displacement=%.3f m" % distance)
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--min-displacement", type=float, default=0.05)
    args = parser.parse_args()
    rclpy.init()
    node = MotionCheck(args.topic, args.seconds, args.min_displacement)
    try:
        rclpy.spin(node)
    finally:
        code = node.result
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(code)


if __name__ == "__main__":
    main()
