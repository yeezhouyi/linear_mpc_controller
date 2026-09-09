#!/usr/bin/env python3
"""A7.2 #5 cross-language projection parity harness.

Generates identical pose sequences on straight / circle / fold trajectories,
runs the C++ windowed chain (dump path REQUIRED as argv[1]; CMake passes
$<TARGET_FILE:projection_parity_dump>) and the Python reference
(mpc_core.frenet.closest_point with the same s_prev chaining) and compares
per-pose (stage, arc, seg, e_y).

A missing dump binary is an ERROR (exit 1); no argument is exit 2.

Usage: python3 benchmark_tools/scripts/projection_parity.py <projection_parity_dump>
"""
from __future__ import annotations

import math
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from mpc_core.frenet import closest_point  # noqa: E402
from mpc_core.types import Trajectory  # noqa: E402


def make_straight(length=8.0, ds=0.02):
    n = int(round(length / ds)) + 1
    s = np.linspace(0.0, length, n)
    return Trajectory(s=s, x=s.copy(), y=np.zeros(n), yaw=np.zeros(n),
                      kappa=np.zeros(n), v=np.full(n, 0.5))


def make_circle(r=2.0, ds=0.02):
    cn = int(round(2.0 * math.pi * r / ds))
    th = np.arange(cn + 1) * ds / r
    x = r * np.cos(th)
    y = r * np.sin(th)
    yaw = th + math.pi / 2.0
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=np.zeros(x.size),
                      v=np.full(x.size, 0.5))


def make_fold(length=2.0, offset=0.30, ds=0.02):
    n = int(round(length / ds)) + 1
    x = np.linspace(0.0, length, n)
    s0 = np.arange(n) * ds
    x2 = x[::-1]
    xs = np.concatenate([x, x2[1:]])
    ys = np.concatenate([np.zeros(n), np.full(n - 1, offset)])
    yaws = np.concatenate([np.zeros(n), np.full(n - 1, math.pi)])
    s = np.zeros(xs.size)
    for i in range(1, xs.size):
        s[i] = s[i - 1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    return Trajectory(s=s, x=xs, y=ys, yaw=yaws, kappa=np.zeros(xs.size),
                      v=np.full(xs.size, 0.5))


def python_chain(traj, poses):
    """Same chaining as the C++ dump: s_prev = previous raw arc (kept on 3)."""
    recs = []
    s_prev = None
    for (px, py, yaw) in poses:
        seg, w, e_y, arc, stage = closest_point(
            traj, px, py, yaw=yaw, s_prev=s_prev)
        recs.append((stage, arc, seg, -999.0 if math.isnan(e_y) else e_y))
        if stage != 3:
            s_prev = arc
    return recs


def run_cpp(bin_path, kind, poses):
    with tempfile.TemporaryDirectory() as td:
        tp = pathlib.Path(td) / "poses.txt"
        op = pathlib.Path(td) / "out.txt"
        tp.write_text("\n".join(f"{p[0]:.9f} {p[1]:.9f} {p[2]:.9f}" for p in poses))
        subprocess.run([str(bin_path), kind, str(tp), str(op)], check=True)
        recs = []
        for line in op.read_text().splitlines():
            st, arc, seg, ey = line.split()
            recs.append((int(st), float(arc), int(seg), float(ey)))
        return recs


def pose_sets():
    def seq(name, traj, maker):
        poses = []
        n = len(traj.s)
        # walk along a lateral-offset lane, plus a jump, plus fold/circle arcs
        if name == "straight":
            for k in range(0, 300, 10):
                poses.append((k * 0.02, 0.05, 0.0))
            poses.append((7.0, 0.05, 0.0))          # jump
        elif name == "circle":
            for k in range(0, n, 30):
                th = traj.s[k] / 2.0
                poses.append((traj.x[k] * 1.0 + 0.05 * math.cos(th + math.pi / 2),
                              traj.y[k] * 1.0 + 0.05 * math.sin(th + math.pi / 2),
                              traj.yaw[k]))
        else:  # fold: drive out, then step toward the middle and across
            for k in range(0, n // 2, 15):
                poses.append((traj.x[k], 0.0, 0.0))
            poses.append((1.0, 0.15, 0.0))           # between lanes
            poses.append((1.0, 0.30, 0.0))           # on the return lane
        return poses

    sets = {
        "straight": (make_straight(), "straight"),
        "circle": (make_circle(), "circle"),
        "fold": (make_fold(), "fold"),
    }
    out = {}
    for name, (traj, kind) in sets.items():
        out[name] = (traj, seq(name, traj, None))
    return out


def main(bin_path):
    if not bin_path.is_file():
        print(f"parity: FAIL dump binary not found: {bin_path}")
        return 1
    total, agree, mismatches = 0, 0, []
    for name, (traj, poses) in pose_sets().items():
        cpp = run_cpp(bin_path, {"straight": "straight", "circle": "circle", "fold": "fold"}[name], poses)
        py = python_chain(traj, poses)
        assert len(cpp) == len(py)
        for i, (c, p) in enumerate(zip(cpp, py)):
            total += 1
            stage_ok = c[0] == p[0]
            arc_ok = abs(c[1] - p[1]) < 1e-6
            ey_ok = abs(c[3] - p[3]) < 1e-6
            if stage_ok and arc_ok and ey_ok:
                agree += 1
            else:
                mismatches.append((name, i, c, p))
    print(f"parity: {agree}/{total} records agree")
    for (name, i, c, p) in mismatches[:12]:
        print(f"  MISMATCH {name}[{i}] cpp(stage={c[0]},arc={c[1]:.4f},ey={c[3]:.4f}) "
              f"py(stage={p[0]},arc={p[1]:.4f},ey={p[3]:.4f})")
    return 0 if agree == total else 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: projection_parity.py <projection_parity_dump>")
        sys.exit(2)
    sys.exit(main(pathlib.Path(sys.argv[1])))
