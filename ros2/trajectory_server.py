#!/usr/bin/env python3
"""trajectory_server: publishes a benchmark track as nav_msgs/Path.

Standalone trajectory source for the standalone_tracking chain (U5/U11):

    linear_mpc_node (~/reference)  <-  trajectory_server (Path)

The full path is republished periodically with a fresh stamp so the
controller's staleness gate (traj_max_age_s) never trips.  Tracks come from
trajectory_tools (straight / circle / s_curve / u_turn).

Usage (sim time):
    ros2 run linear_mpc_controller trajectory_server \
        --ros-args -p track:=circle -p rate_hz:=2.0 -p use_sim_time:=true
"""
import os
import sys
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Make the source-layout packages importable when run from the repo
# (colcon-installed deployments should provide trajectory_tools on sys.path).
# resolve() follows the colcon symlink back to the real repo root
_ROOT = os.path.dirname(os.path.dirname(
    os.path.realpath(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if "/home/zhouyi/ros2_ws/src/linear_mpc_controller" not in sys.path and os.path.isdir(
    "/home/zhouyi/ros2_ws/src/linear_mpc_controller"
):
    sys.path.insert(0, "/home/zhouyi/ros2_ws/src/linear_mpc_controller")


class TrajectoryServer(Node):
    def __init__(self):
        super().__init__("trajectory_server")
        self.declare_parameter("track", "circle")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("topic", "reference")
        self.declare_parameter("path_file", "")
        track = self.get_parameter("track").value
        rate = self.get_parameter("rate_hz").value
        topic = self.get_parameter("topic").value
        path_file = self.get_parameter("path_file").value

        self._traj = None
        if path_file:
            # B4/B6: serve a recorded explorer path (record_explorer_path.py
            # JSON) instead of a synthetic benchmark track (AE9 contract).
            self._path = self._path_from_file(path_file)
            self._traj = None
        else:
            from trajectory_tools.reference_trajectory import (
                generate_benchmark_tracks,
            )

            tracks = generate_benchmark_tracks()
            if track not in tracks:
                self.get_logger().error(
                    f"unknown track {track}; use one of {list(tracks)}")
                raise SystemExit(2)
            self._traj = tracks[track]
            self._path = self._to_path()
        self._pub = self.create_publisher(Path, topic, 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self._publish()
        n_pts = len(self._path.poses)
        source = path_file if path_file else track
        length = (f"{self._traj.total_length:.2f} m" if self._traj is not None
                  else "recorded")
        self.get_logger().info(
            f"publishing '{source}' ({n_pts} pts, len {length}) on '{topic}'")

    def _path_from_file(self, path_file: str) -> Path:
        import json

        import numpy as np

        _scripts = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(os.path.abspath(__file__)))),
            "benchmark_tools", "scripts")
        sys.path.insert(0, _scripts)
        from replay_path_mpc import build_trajectory

        rec = json.loads(open(path_file).read())
        traj = build_trajectory(rec["poses"])
        path = Path()
        path.header.frame_id = "odom"
        for i in range(traj.n):
            p = PoseStamped()
            p.header.frame_id = "odom"
            p.pose.position.x = float(traj.x[i])
            p.pose.position.y = float(traj.y[i])
            yaw = float(traj.yaw[i])
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(p)
        self.get_logger().info(
            f"loaded recorded path: {len(rec['poses'])} poses -> "
            f"{traj.n} reference points")
        return path

    def _to_path(self):
        path = Path()
        path.header.frame_id = "odom"
        for i in range(self._traj.n):
            p = PoseStamped()
            p.header.frame_id = "odom"
            p.pose.position.x = float(self._traj.x[i])
            p.pose.position.y = float(self._traj.y[i])
            yaw = float(self._traj.yaw[i])
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(p)
        return path

    def _publish(self):
        self._path.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._path)


def main():
    rclpy.init()
    node = TrajectoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
