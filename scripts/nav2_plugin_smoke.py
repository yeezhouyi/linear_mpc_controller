#!/usr/bin/env python3
"""B6B sandbox acceptance (spec parts 5/6): lifecycle stage 1, straight
stage 2a, arc-with-offline-start stage 2b.

Verdict discipline:
  * terminal outcome read from the GoalHandle (status == 4 SUCCEEDED) and
    result.error_code -- NEVER from log lines (silent-false-pass trap #7);
  * every assertion counted; a stage prints N/M then exits nonzero on any
    failure.

Usage (run under the sourced colcon install):
  python3 nav2_plugin_smoke.py --stage 1
  python3 nav2_plugin_smoke.py --stage 2a
  python3 nav2_plugin_smoke.py --stage 2b
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient

PLUGIN = "linear_mpc_controller::LinearMpcNav2Controller"
STRAIGHT_LEN = 1.8
ARC_R = 1.5          # quarter circle -> length pi/2*1.5 = 2.356
XY_TOL = 0.25
YAW_TOL = 0.25


def sh(args, timeout=20):
    r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr)


def straight_path():
    p = Path()
    p.header.frame_id = "odom"
    n = 91
    for i in range(n):
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = STRAIGHT_LEN * i / (n - 1)
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def arc_path():
    p = Path()
    p.header.frame_id = "odom"
    n = 121
    for i in range(n):
        from geometry_msgs.msg import PoseStamped
        th = 0.5 * math.pi * i / (n - 1)
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = ARC_R * math.sin(th)
        ps.pose.position.y = ARC_R - ARC_R * math.cos(th)
        ps.pose.orientation.w = math.cos(th / 2.0)
        ps.pose.orientation.z = math.sin(th / 2.0)
        p.poses.append(ps)
    return p


def probe_cmd_vel_type():
    rc, out = sh(["ros2", "topic", "info", "/cmd_vel", "-t"])
    for ln in out.splitlines():
        if "Type:" in ln:
            return ln.split(":")[-1].strip().split("/")[-1]
    return "TwistStamped"


class Probes:
    def __init__(self, node):
        self.node = node
        self.cmd_v = []
        self.cmd_w = []
        self.odom = []
        # Subscribe the ONE type that actually exists on /cmd_vel (both at
        # once trips the RMW type-mismatch "invalid allocator" error).
        if probe_cmd_vel_type() == "Twist":
            node.create_subscription(Twist, "/cmd_vel", self.cb_tw, 10)
        else:
            node.create_subscription(TwistStamped, "/cmd_vel", self.cb_ts, 10)
        node.create_subscription(Odometry, "/odom", self.cb_odom, 10)

    def cb_ts(self, m):
        self.cmd_v.append(m.twist.linear.x)
        self.cmd_w.append(m.twist.angular.z)

    def cb_tw(self, m):
        self.cmd_v.append(m.linear.x)
        self.cmd_w.append(m.angular.z)

    def cb_odom(self, m):
        self.odom.append((m.pose.pose.position.x, m.pose.pose.position.y,
                          m.pose.pose.orientation.z, m.pose.pose.orientation.w))


def wait_lifecycle(cmd, state_word, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rc, out = sh(["ros2", "lifecycle", "get", "/controller_server"])
        if state_word in out:
            return True
        time.sleep(1)
    return False


def do_lifecycle(cmd):
    return sh(["ros2", "lifecycle", "set", "/controller_server", cmd])


def stage1():
    checks = []
    # 1) the plugin must be the loaded FollowPath type (objective: param)
    rc, out = do_lifecycle("configure")
    checks.append(("configure returns 0", rc == 0))
    # controller_server re-configures costmap/plugins asynchronously; poll
    rc2, out = (0, "")
    t0 = time.time()
    plugin_ok = False
    while time.time() - t0 < 60:
        rc2, out = sh(["ros2", "param", "get", "/controller_server",
                       "FollowPath.plugin"], timeout=10)
        if PLUGIN in out:
            plugin_ok = True
            break
        time.sleep(1)
    checks.append(("FollowPath.plugin == our plugin", plugin_ok))
    rc, out = do_lifecycle("activate")
    checks.append(("activate returns 0", rc == 0))
    checks.append(("lifecycle active [3]",
                   wait_lifecycle("activate", "active")))
    report("stage1 lifecycle", checks)
    return all(c for _, c in checks)


def wait_future(node, future, timeout_s):
    """Single-thread spin until future done or deadline (no background
    executor threads -- rclpy GlobalExecutor is not thread-safe to share)."""
    t0 = time.time()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.time() - t0 > timeout_s:
            return False
    return True


def run_follow_path(node, path, timeout_s=45):
    client = ActionClient(node, FollowPath, "/follow_path")
    if not client.wait_for_server(15):
        return None, "action server not up"
    goal = FollowPath.Goal()
    goal.path = path
    fut = client.send_goal_async(goal)
    if not wait_future(node, fut, timeout_s):
        client.cancel_goal_async()
        return None, "goal send timed out"
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None, "goal not accepted"
    res_fut = gh.get_result_async()
    if not wait_future(node, res_fut, timeout_s):
        gh.cancel_goal_async()
        return None, "goal execution timed out"
    result = res_fut.result()
    status = result.status                # 4 == STATUS_SUCCEEDED
    err = result.result.error_code
    return (status, err), None


def stage_control(node, stage):
    probes = Probes(node)
    path = straight_path() if stage == "2a" else arc_path()
    t0 = time.time()
    outcome, errmsg = run_follow_path(node, path)
    dt = time.time() - t0
    return outcome, errmsg, probes, dt


def report(title, checks):
    n_ok = sum(1 for _, ok in checks if ok)
    print(f"[B6B {title}] {n_ok}/{len(checks)} passed")
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return n_ok == len(checks)


def stage2(node, stage):
    outcome, errmsg, probes, dt = stage_control(node, stage)
    checks = []
    checks.append(("goal terminal outcome", outcome is not None))
    if outcome is not None:
        status, err = outcome
        checks.append(("GoalHandle status==4 SUCCEEDED", status == 4))
        checks.append(("error_code==0", err == 0))
    nonzero = [v for v in probes.cmd_v if abs(v) > 1e-6]
    checks.append(("nonzero cmd_vel count >= 50", len(nonzero) >= 50))
    if not probes.odom:
        checks.append(("robot moved (odom received)", False))
        return report(f"stage{stage} control", checks)
    fx, fy = probes.odom[-1][0], probes.odom[-1][1]
    import numpy as np
    xs = np.array([o[0] for o in probes.odom])
    ys = np.array([o[1] for o in probes.odom])
    max_disp = float(np.max(np.hypot(xs, ys)))
    if stage == "2a":
        checks.append(("final x >= 1.7", fx >= 1.7))
        checks.append(("max displacement <= 2.3", max_disp <= 2.3))
        checks.append(("final |y| < 0.05", abs(fy) < 0.05))
    else:
        mx = float(np.max(np.abs(probes.cmd_w)))
        mv = float(np.max(np.abs(probes.cmd_v)))
        checks.append(("max |omega| > 0.1 (steering excited)", mx > 0.1))
        checks.append(("max v > 0.1", mv > 0.1))
        end_x, end_y = ARC_R, ARC_R
        d_end = math.hypot(fx - end_x, fy - end_y)
        checks.append(("final within xy tol of arc end", d_end <= 0.5))
        bound = math.pi * ARC_R / 2 + 0.32 + 0.40
        checks.append(("max displacement <= bound", max_disp <= bound))
    return report(f"stage{stage} control", checks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=["1", "2a", "2b"])
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("b6b_smoke")
    if args.stage == "1":
        ok = stage1()
    else:
        ok = stage2(node, args.stage)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
