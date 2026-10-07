#!/usr/bin/env python3
"""Compare one complete C++ controller cycle with the Python repair tree."""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPAIR = Path("/home/zhouyi/mpc_review_worktrees/r6_core")
if str(REPAIR) not in sys.path:
    sys.path.insert(0, str(REPAIR))
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from mpc_core.mpc import LinearMpcController  # noqa: E402
from mpc_core.qp_osqp import OsqpQp  # noqa: E402
from mpc_core.types import KinematicState, MpcParams, TrackPointKind, Trajectory  # noqa: E402


PI = math.pi


def straight():
    s = np.arange(121, dtype=float) * 0.05
    return s, s.copy(), np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), np.full_like(s, 0.15)


def arc(left: bool):
    radius = 3.0
    sign = 1.0 if left else -1.0
    theta = np.linspace(0.0, 1.2, 121)
    return (radius * theta, radius * np.sin(theta),
            sign * radius * (1.0 - np.cos(theta)), sign * theta,
            np.full_like(theta, sign / radius), np.full_like(theta, 0.15))


def s_curve():
    length = 6.0
    amplitude = 0.25
    s = np.linspace(0.0, length, 241)
    w = 2.0 * PI / length
    dy = amplitude * w * np.cos(w * s)
    ddy = -amplitude * w * w * np.sin(w * s)
    return s, s.copy(), amplitude * np.sin(w * s), np.arctan(dy), ddy / (1.0 + dy * dy) ** 1.5, np.full_like(s, 0.15)


def make_params() -> MpcParams:
    return MpcParams(
        Ts=0.05, N=25, Q_diag=(60.0, 10.0, 2.0, 1.0),
        Q_F_diag=(120.0, 20.0, 4.0, 2.0), S_diag=(0.5, 0.5),
        v_min=0.0, v_max=0.30, omega_max=1.20, a_max=0.65,
        alpha_max=2.0, lookahead_m=0.0, qp_timeout_s=0.0,
    )


def python_rows():
    cases = []
    s, x, y, yaw, kappa, vref = straight()
    cases.append(("straight", (0.80, 0.04, 0.02, 0.15, 0.01), (s, x, y, yaw, kappa, vref)))
    s, x, y, yaw, kappa, vref = arc(True)
    t = 0.315
    cases.append(("left_arc", (3.0 * math.sin(t) - 0.03 * math.sin(t),
                                3.0 * (1.0 - math.cos(t)) + 0.03 * math.cos(t),
                                t + 0.02, 0.15, 0.15 / 3.0 + 0.01),
                  (s, x, y, yaw, kappa, vref)))
    s, x, y, yaw, kappa, vref = arc(False)
    cases.append(("right_arc", (3.0 * math.sin(t) + 0.03 * math.sin(t),
                                 -3.0 * (1.0 - math.cos(t)) - 0.03 * math.cos(t),
                                 -t - 0.02, 0.15, -0.15 / 3.0 - 0.01),
                  (s, x, y, yaw, kappa, vref)))
    s, x, y, yaw, kappa, vref = s_curve()
    ss = 2.4
    w = 2.0 * PI / 6.0
    dy = 0.25 * w * math.cos(w * ss)
    ddy = -0.25 * w * w * math.sin(w * ss)
    heading = math.atan(dy)
    cases.append(("s_curve", (ss - 0.03 * math.sin(heading),
                              0.25 * math.sin(w * ss) + 0.03 * math.cos(heading),
                              heading + 0.02, 0.15,
                              ddy / (1.0 + dy * dy) ** 1.5 * 0.15 + 0.01),
                  (s, x, y, yaw, kappa, vref)))
    return cases


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_controller_parity.py <controller_parity_dump>", file=sys.stderr)
        return 2
    proc = subprocess.run([sys.argv[1]], text=True, capture_output=True, check=False)
    print(proc.stdout, end="")
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr)
        return proc.returncode
    cpp = {}
    for line in proc.stdout.splitlines():
        fields = line.strip().split(",")
        if len(fields) != 12:
            continue
        cpp[fields[0]] = np.asarray([float(v) for v in fields[1:]], dtype=float)
    expected_names = {name for name, _, _ in python_rows()}
    if set(cpp) != expected_names:
        print(f"FAIL cases: C++={sorted(cpp)} expected={sorted(expected_names)}")
        return 1

    failures = []
    for name, state, arrays in python_rows():
        s, x, y, yaw, kappa, vref = arrays
        traj = Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=kappa, v=vref,
                          kind=TrackPointKind.WITH_VELOCITY_CURVATURE)
        controller = LinearMpcController(make_params(), traj, solver=OsqpQp(
            max_iter=1500, abs_tol=1e-6, rel_tol=1e-5, polish=True))
        out = controller.compute_cycle(KinematicState(
            x=state[0], y=state[1], yaw=state[2], v=state[3], omega=state[4]))
        py = np.asarray([
            float(out.diag.health),
            0.0 if out.diag.qp_status == "SOLVED" else 1.0,
            out.diag.a0, out.diag.alpha0, out.v_cmd, out.omega_cmd,
            *out.diag.e_ref, out.diag.accepted_arc,
        ], dtype=float)
        row = cpp[name]
        # Health/status must match exactly; numerical values use a fixed
        # tolerance appropriate for the two independently built dense QPs.
        if int(row[0]) != int(py[0]) or int(row[1]) != int(py[1]):
            failures.append(f"{name}: status cpp={row[:2]} py={py[:2]}")
            continue
        diff = np.abs(row[2:] - py[2:])
        control_max = float(np.max(diff[:4]))
        diagnostic_max = float(np.max(diff[4:]))
        print(f"PARITY {name} control_diff={diff[:4].tolist()} control_max={control_max:.9e} diagnostic_max={diagnostic_max:.9e}")
        if control_max > 1.0e-6 or diagnostic_max > 2.0e-5:
            failures.append(
                f"{name}: control_max={control_max:.3e} diagnostic_max={diagnostic_max:.3e}")
    if failures:
        print("FAIL controller parity")
        print("\n".join(failures))
        return 1
    print("PASS complete controller parity: 4 fixtures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
