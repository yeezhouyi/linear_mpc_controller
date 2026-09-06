#!/usr/bin/env python3
"""B4 bridge (replay side): feed a recorded explorer path through the MPC
reference core (offline, pure-python) and produce tracking metrics.

Path completion (deterministic, mirrors ros2/trajectory_adapter.cpp):
  * yaw from neighbour chords, curvature from heading deltas over 2-chord
    arc,
  * speed: v_default capped by curve speed and v_max.

Usage:
  python3 replay_path_mpc.py --recorded explorer_path.json \
      --output replay_metrics.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from mpc_core.frenet import closest_point
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import KinematicState, MpcParams, Trajectory, wrap_angle


def build_trajectory(poses) -> Trajectory:
    """poses: [[x, y, ...], ...] (yaw/t ignored; recomputed deterministically)."""
    xy = [(float(p[0]), float(p[1])) for p in poses]
    if len(xy) < 2:
        raise ValueError("recorded path needs >= 2 poses")
    # drop duplicates (recorder already min-steps, but be safe)
    dedup = [xy[0]]
    for p in xy[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-6:
            dedup.append(p)
    n = len(dedup)
    xs = np.array([p[0] for p in dedup])
    ys = np.array([p[1] for p in dedup])
    s = np.zeros(n)
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    yaw = np.zeros(n)
    for i in range(n):
        j0 = max(0, i - 1)
        j1 = min(n - 1, i + 1)
        yaw[i] = math.atan2(ys[j1] - ys[j0], xs[j1] - xs[j0])
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        arc = 0.5 * ((s[i] - s[i - 1]) + (s[i + 1] - s[i]))
        if arc > 1e-9:
            kappa[i] = wrap_angle(yaw[i + 1] - yaw[i - 1]) / (2.0 * arc)
    # speed completion with a floor: sharp recorded corners give huge local
    # kappa -> v_ref ~ 0 -> the tracker stalls forever at the corner
    v_floor = 0.15
    v = np.full(n, 0.5)
    for i in range(n):
        k = abs(kappa[i])
        if k > 1e-6:
            v[i] = max(v_floor, min(0.5, 0.3 / max(k, 1e-6)))
    return Trajectory(s=s, x=xs, y=ys, yaw=yaw, kappa=kappa, v=v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recorded", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--v-max", type=float, default=1.5)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    args = ap.parse_args()

    rec = json.loads(Path(args.recorded).read_text())
    poses = rec["poses"]
    traj = build_trajectory(poses)

    controller = LinearMpcController(MpcParams(N=25), traj)
    controller.set_reference(traj)
    x0, y0 = float(poses[0][0]), float(poses[0][1])
    yaw0 = float(poses[0][2]) if len(poses[0]) > 2 else 0.0
    plant = DifferentialDrivePlant(x0=x0, y0=y0, yaw0=yaw0, v0=0.0, omega0=0.0)

    Ts = 0.05
    e_y, e_psi, dv = [], [], []
    prev_cmd = np.zeros(2)
    reached = False
    steps = int(args.timeout_s / Ts)
    for _ in range(steps):
        st = plant.state
        _, _, e, arc = closest_point(traj, st.x, st.y)
        if arc >= traj.total_length - 0.10:
            reached = True
            break
        anchor = traj.sample_by_s(arc)
        e_psi_v = wrap_angle(st.yaw - anchor.yaw)
        out = controller.compute_cycle(
            KinematicState(x=st.x, y=st.y, yaw=st.yaw, v=st.v, omega=st.omega))
        cmd_v = min(max(out.v_cmd, 0.0), args.v_max)
        cmd_w = max(min(out.omega_cmd, 2.0), -2.0)
        plant.step(cmd_v, cmd_w, Ts)
        e_y.append(abs(e))
        e_psi.append(abs(e_psi_v))
        dv.append((cmd_v - prev_cmd[0]) ** 2 + (cmd_w - prev_cmd[1]) ** 2)
        prev_cmd = np.array([cmd_v, cmd_w])

    e_y_arr = np.array(e_y) if e_y else np.array([0.0])
    metrics = {
        "path_length_m": round(float(traj.total_length), 3),
        "recorded_poses": len(poses),
        "completed": bool(reached),
        "steps": len(e_y),
        "e_y_rms": round(float(np.sqrt(np.mean(e_y_arr ** 2))), 4),
        "e_y_p95": round(float(np.percentile(e_y_arr, 95)), 4),
        "e_y_max": round(float(e_y_arr.max()), 4),
        "smooth_du": round(float(np.mean(dv)) if dv else 0.0, 6),
    }
    Path(args.output).write_text(json.dumps(metrics, indent=1))
    print(json.dumps(metrics, indent=1))


if __name__ == "__main__":
    main()
