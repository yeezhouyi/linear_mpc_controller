#!/usr/bin/env python3
"""TB3 Gazebo closed-loop smoke checker (P0: 闭环冒烟).

Subscribes /odom and the reference Path for LOOK_S seconds, then reports:
  samples, driven length, distance-to-reference median/p95/max,
  fraction of samples within 0.35 m of the reference, mean speed.
Writes JSON to the path given as argv[1].  PASS when median <= 0.30 m,
in-band fraction >= 0.85 and driven length >= 3 m.
"""
import json
import math
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

LOOK_S = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tb3_smoke.json"


class Smoke(Node):
    def __init__(self):
        super().__init__("tb3_smoke_checker")
        self.path_pts = None
        self.odom = []
        self.path_evt = threading.Event()
        self.create_subscription(Path, "/linear_mpc_node/reference", self.on_path, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)

    def on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y)
               for p in msg.poses]
        if len(pts) >= 2:
            self.path_pts = pts
            self.path_evt.set()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((p.x, p.y, time.monotonic()))


def main():
    rclpy.init()
    node = Smoke()
    t0 = time.monotonic()
    # wait for the reference path (server publishes at 2 Hz)
    while not node.path_evt.is_set() and time.monotonic() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.path_pts is None:
        json.dump(dict(result="FAIL", reason="no reference path received"),
                  open(OUT, "w"))
        return
    # collect odom
    end = time.monotonic() + LOOK_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()

    pts = node.path_pts
    dist = []
    prev = None
    driven = 0.0
    last = None
    for (x, y, _t) in node.odom:
        dmin = min(math.hypot(x - px, y - py) for (px, py) in pts)
        dist.append(dmin)
        if last is not None:
            driven += math.hypot(x - last[0], y - last[1])
        last = (x, y)
    med = sorted(dist)[len(dist) // 2] if dist else 999
    p95 = sorted(dist)[int(len(dist) * 0.95)] if dist else 999
    inband = sum(1 for d in dist if d <= 0.35) / max(1, len(dist))
    speed = driven / LOOK_S
    ok = med <= 0.30 and inband >= 0.85 and driven >= 3.0
    result = dict(
        result="PASS" if ok else "FAIL",
        samples=len(node.odom),
        path_points=len(pts),
        driven_m=round(driven, 2),
        mean_speed_mps=round(speed, 3),
        dist_median_m=round(float(med), 4),
        dist_p95_m=round(float(p95), 4),
        in_band_frac=round(inband, 4),
        thresholds=dict(median_le_m=0.30, in_band_ge=0.85, driven_ge_m=3.0),
    )
    json.dump(result, open(OUT, "w"), indent=1)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
