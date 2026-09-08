#!/usr/bin/env python3
"""B6B sandbox acceptance (spec parts 5/6): lifecycle stage 1, straight
stage 2a, arc-with-offline-start stage 2b.

Verdict discipline:
  * terminal outcome read from the GoalHandle (status == 4 SUCCEEDED) and
    result.error_code -- NEVER from log lines (silent-false-pass trap #7);
  * every assertion counted; a stage prints N/M then exits nonzero on any
    failure;
  * NO ros2 CLI anywhere: lifecycle transitions and the plugin-param check
    go through typed in-process service clients (ChangeState / GetState /
    GetParameters), and the goal goes through rclpy ActionClient.  The CLI
    sits on the daemon, whose graph cache makes "action up" answers stale
    and goal delivery run-to-run nondeterministic (B6B review).

Usage (run under the sourced colcon install):
  python3 nav2_plugin_smoke.py --stage 1
  python3 nav2_plugin_smoke.py --stage 2a
  python3 nav2_plugin_smoke.py --stage 2b
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient

PLUGIN = "linear_mpc_controller::LinearMpcNav2Controller"
NODE = "/controller_server"
STRAIGHT_LEN = 1.8
ARC_R = 1.5          # quarter circle -> length pi/2*1.5 = 2.356
XY_TOL = 0.25
YAW_TOL = 0.25
ACTIVE = State.PRIMARY_STATE_ACTIVE  # 3


def straight_path():
    from geometry_msgs.msg import PoseStamped
    p = Path()
    p.header.frame_id = "odom"
    n = 91
    for i in range(n):
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = STRAIGHT_LEN * i / (n - 1)
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def arc_path():
    from geometry_msgs.msg import PoseStamped
    p = Path()
    p.header.frame_id = "odom"
    n = 121
    for i in range(n):
        th = 0.5 * math.pi * i / (n - 1)
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = ARC_R * math.sin(th)
        ps.pose.position.y = ARC_R - ARC_R * math.cos(th)
        ps.pose.orientation.w = math.cos(th / 2.0)
        ps.pose.orientation.z = math.sin(th / 2.0)
        p.poses.append(ps)
    return p


def arc_nc_path():
    """Negative-control path: the SAME arc geometry but every pose carries a
    UNIT quaternion (B6B review).  adaptPath derives yaw from geometry
    (atan2), so tracking is unchanged -- but the goal checker judges the
    LAST pose's yaw: the robot arrives at the arc-end TANGENT heading
    (~pi/2), the goal demands 0, yaw_goal_tolerance (0.25) can never be
    met, so the goal can never self-succeed.  Only the terminal-stop clamp
    then stops the robot from driving past the end -- this scenario is what
    actually loads the clamp (a tangent-yaw path would let the goal checker
    succeed first: a false pass)."""
    from geometry_msgs.msg import PoseStamped
    p = Path()
    p.header.frame_id = "odom"
    n = 121
    for i in range(n):
        th = 0.5 * math.pi * i / (n - 1)
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = ARC_R * math.sin(th)
        ps.pose.position.y = ARC_R - ARC_R * math.cos(th)
        ps.pose.orientation.w = 1.0          # unit quaternion, yaw = 0
        p.poses.append(ps)
    return p


class Probes:
    """cmd_vel flavour is PINNED to plain Twist (yaml
    enable_stamped_cmd_vel: false) -- no probing, matching the robot."""

    def __init__(self, node):
        self.node = node
        self.cmd_v = []
        self.cmd_w = []
        self.odom = []
        node.create_subscription(Twist, "/cmd_vel", self.cb_tw, 10)
        node.create_subscription(Odometry, "/odom", self.cb_odom, 10)

    def cb_tw(self, m):
        self.cmd_v.append(m.linear.x)
        self.cmd_w.append(m.angular.z)

    def cb_odom(self, m):
        self.odom.append((m.pose.pose.position.x, m.pose.pose.position.y,
                          m.pose.pose.orientation.z, m.pose.pose.orientation.w))


def wait_future(node, future, timeout_s):
    """Single-thread spin until future done or deadline (no background
    executor threads -- rclpy GlobalExecutor is not thread-safe to share)."""
    t0 = time.time()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.time() - t0 > timeout_s:
            return False
    return True


def call_service(node, client, req, timeout_s=20):
    if not client.wait_for_service(timeout_s):
        return None
    fut = client.call_async(req)
    if not wait_future(node, fut, timeout_s):
        return None
    return fut.result()


def bringup(node):
    """Lifecycle configure -> plugin-param assert -> activate, all through
    typed in-process service clients.  Returns (checks, all_ok)."""
    checks = []
    chg = node.create_client(ChangeState, NODE + "/change_state")
    get = node.create_client(GetState, NODE + "/get_state")
    prm = node.create_client(GetParameters, NODE + "/get_parameters")

    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    res = call_service(node, chg, req, 20)
    checks.append(("configure transition served", res is not None and res.success))

    # the plugin declares its params during configure; poll until the value
    # lands (server configures costmap/plugins asynchronously).
    plugin_ok = False
    t0 = time.time()
    while time.time() - t0 < 60:
        pr = GetParameters.Request()
        pr.names = ["FollowPath.plugin"]
        rr = call_service(node, prm, pr, 10)
        if rr is not None and rr.values and \
                rr.values[0].string_value == PLUGIN:
            plugin_ok = True
            break
        time.sleep(1)
    checks.append(("FollowPath.plugin == our plugin", plugin_ok))

    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_ACTIVATE
    res = call_service(node, chg, req, 20)
    checks.append(("activate transition served", res is not None and res.success))

    active = False
    t0 = time.time()
    while time.time() - t0 < 15:
        rr = call_service(node, get, GetState.Request(), 10)
        if rr is not None and rr.current_state.id == ACTIVE:
            active = True
            break
        time.sleep(0.5)
    checks.append(("lifecycle active", active))
    return checks, all(ok for _, ok in checks)


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
    else:
        checks.append(("errmsg", True))
        print(f"    (goal error: {errmsg})")
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
    print(f"    diag: final=({fx:.3f},{fy:.3f}) max_disp={max_disp:.3f} "
          f"n_cmd={len(probes.cmd_v)} n_odom={len(probes.odom)}")
    if stage == "2a":
        # The plugin's terminal_stop_margin (0.20 m) deliberately stops the
        # robot short of the plan end; the goal checker then declares SUCCESS
        # once the robot is inside xy_goal_tolerance (0.25) of the end pose.
        # So final x sits in [len-margin-tol, len+tol] ~ [1.35, 2.05]; asking
        # for >= 1.7 would contradict the margin's own design.
        checks.append(("final x within [1.35, 2.05] of end", 1.35 <= fx <= 2.05))
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


def stage_nc(node, timeout_s=32):
    """Negative-control scenario runner.  Prints NC_MAXDISP=<max_disp> for
    the shell runner to compare intact vs clamp-disabled builds.  Verdicts
    here are only about scenario sanity; the causal claim (clamp stops the
    overshoot) is the runner's B - A >= 2.0 m comparison.

    Terminal semantics differ from the control stages ON PURPOSE: the intact
    build stops short and the progress checker ABORTS the goal; the patched
    build never stops, so the smoke cancels it at timeout.  BOTH are
    legitimate outcomes here -- what must never happen is a SUCCEEDED (the
    unit-quat yaw must keep the goal checker unsatisfiable), and what must
    never be returned is a harness-level failure (no server / goal not
    accepted / send timeout)."""
    probes = Probes(node)
    path = arc_nc_path()
    outcome, errmsg = run_follow_path(node, path, timeout_s=timeout_s)
    status, err = (None, None) if outcome is None else outcome
    timed_out = errmsg == "goal execution timed out"
    max_disp = 0.0
    checks = []
    checks.append(
        ("terminal or timeout reached (no harness failure)",
         outcome is not None or timed_out))
    checks.append(
        ("goal never SUCCEEDED (unit-quat yaw unsatisfiable)",
         status != 4))
    checks.append(("cmd_vel flowed", len(probes.cmd_v) > 10))
    if probes.odom:
        import numpy as np
        xs = np.array([o[0] for o in probes.odom])
        ys = np.array([o[1] for o in probes.odom])
        max_disp = float(np.max(np.hypot(xs, ys)))
        checks.append(("robot moved (odom received)", True))
        print(f"    diag: final=({probes.odom[-1][0]:.3f},{probes.odom[-1][1]:.3f}) "
              f"max_disp={max_disp:.3f} n_cmd={len(probes.cmd_v)} "
              f"n_odom={len(probes.odom)}")
    else:
        checks.append(("robot moved (odom received)", False))
    print(f"NC_MAXDISP={max_disp:.3f}")
    return report("stage nc scenario", checks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=["1", "2a", "2b", "nc"])
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("b6b_smoke")
    bchecks, bok = bringup(node)
    if args.stage == "1":
        ok = report("stage1 lifecycle", bchecks)
    elif args.stage == "nc":
        ok_b = report(f"stage nc bringup", bchecks)
        ok_c = stage_nc(node)
        ok = ok_b and ok_c
    else:
        ok_b = report(f"stage{args.stage} bringup", bchecks)
        ok_c = stage2(node, args.stage)
        ok = ok_b and ok_c
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
