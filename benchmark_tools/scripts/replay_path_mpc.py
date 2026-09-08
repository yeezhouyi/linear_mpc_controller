#!/usr/bin/env python3
"""B4 bridge (replay side): feed a recorded explorer path through the MPC
reference core (offline, pure-python) and produce tracking metrics.

Path completion (deterministic, mirrors ros2/trajectory_adapter.cpp):
  * RESAMPLE uniformly by arc length first (ds=0.05) -- recorded plans are
    non-uniform and finite-difference curvature on the raw polyline
    manufactures phantom corners (Day 4-5);
  * yaw from neighbour chords, curvature from heading deltas over 2-chord
    arc (on the resampled points);
  * speed: v = min(v_default, omega_max/|kappa|); the 0.15 floor applies
    only where |kappa| <= omega_max/0.15, never pushing above omega_max.

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
from mpc_core.progress_gate import ProgressAllowanceGate
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import KinematicState, MpcParams, Trajectory, wrap_angle
from trajectory_tools.curvature_estimator import (
    estimate_curvature, estimate_heading, omega_violations,
)
from trajectory_tools.resample import resample_uniform


def build_trajectory(poses) -> Trajectory:
    """poses: [[x, y, ...], ...] (yaw/t ignored; recomputed deterministically).

    Day 4-5: resample uniformly (ds=0.05) BEFORE deriving yaw/kappa/speed,
    then complete speed kinematically (omega_max-bounded; floor only in the
    legal band).  Guards that the finished reference obeys omega_max."""
    xy = [(float(p[0]), float(p[1])) for p in poses]
    if len(xy) < 2:
        raise ValueError("recorded path needs >= 2 poses")
    # drop duplicates (recorder already min-steps, but be safe)
    dedup = [xy[0]]
    for p in xy[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-6:
            dedup.append(p)
    xs, ys = resample_uniform(
        [p[0] for p in dedup], [p[1] for p in dedup], ds=0.05)
    n = xs.size
    yaw = estimate_heading(xs, ys)
    kappa = estimate_curvature(xs, ys)
    s = np.zeros(n)
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    # kinematic completion: v = min(v_default, omega_max/|kappa|); floor only
    # where |kappa| <= omega_max/v_floor (never pushes above omega_max).
    omega_max = 2.0
    v_floor = 0.15
    k_abs = np.abs(kappa)
    eps = 1e-6
    v = np.full(n, 0.5)
    with np.errstate(divide="ignore", invalid="ignore"):
        cap = np.where(k_abs > eps, omega_max / np.maximum(k_abs, eps), 0.5)
    v = np.minimum(v, cap)
    legal = k_abs <= omega_max / max(v_floor, 1e-9)
    v = np.where(legal, np.maximum(v, v_floor), v)
    viol, ratio = omega_violations(kappa, v, omega_max)
    if viol.size:
        raise RuntimeError(
            f"completed reference violates omega bound: {viol.size} pts, "
            f"max ratio {ratio:.3f} (guard)")
    return Trajectory(s=s, x=xs, y=ys, yaw=yaw, kappa=kappa, v=v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recorded", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--v-max", type=float, default=1.5)
    ap.add_argument("--v-min", type=float, default=0.0,
                    help="A8 matrix axis: velocity lower bound")
    ap.add_argument("--controller-projection-mode", choices=("windowed", "global"),
                    default="windowed",
                    help="A8 matrix axis: A5 gated vs pre-A5 raw-arc controller")
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--qp-max-iter", type=int, default=500,
                    help="offline evaluation needs convergence, not the "
                         "1500-iter precision cap; noisy recorded paths "
                         "make the ADMM hit the cap (~0.3 s/step at 1500)")
    args = ap.parse_args()

    rec = json.loads(Path(args.recorded).read_text())
    poses = rec["poses"]
    traj = build_trajectory(poses)

    controller = LinearMpcController(
        MpcParams(N=25, qp_max_iter=args.qp_max_iter,
                  v_min=args.v_min,
                  controller_projection_mode=args.controller_projection_mode),
        traj)
    controller.set_reference(traj)
    x0, y0 = float(poses[0][0]), float(poses[0][1])
    yaw0 = float(poses[0][2]) if len(poses[0]) > 2 else 0.0
    plant = DifferentialDrivePlant(x0=x0, y0=y0, yaw0=yaw0, v0=0.0, omega0=0.0)

    Ts = 0.05
    e_y, e_psi, dv = [], [], []
    prev_cmd = np.zeros(2)
    reached = False
    raw_arc_final = 0.0
    acc_arc_final = 0.0
    high_watermark = 0.0
    jump_episodes = 0
    # A7.3: the replay ledger routes through the shared gate; completion and
    # progress read the ACCEPTED arc, never the raw projection.
    st0 = plant.state
    _, _, _, seed_arc, _ = closest_point(traj, st0.x, st0.y)
    gate = ProgressAllowanceGate(v_max=args.v_max, Ts=Ts,
                                 accepted_arc=float(seed_arc))
    gate.step(traj, st0.x, st0.y, float(seed_arc))  # prime (first call anchors)
    in_jump = False
    steps = int(args.timeout_s / Ts)
    for _ in range(steps):
        st = plant.state
        _, _, e, arc, _ = closest_point(traj, st.x, st.y)
        raw_arc_final = float(arc)
        dec = gate.step(traj, st.x, st.y, arc)
        if not dec.accepted and not in_jump:
            in_jump = True
            jump_episodes += 1
        elif dec.accepted:
            in_jump = False
        acc_arc_final = float(gate.accepted_arc)
        high_watermark = max(high_watermark, acc_arc_final)
        if acc_arc_final >= traj.total_length - 0.10:
            reached = True
            break
        anchor = traj.sample_by_s(arc)
        e_psi_v = wrap_angle(st.yaw - anchor.yaw)
        out = controller.compute_cycle(
            KinematicState(x=st.x, y=st.y, yaw=st.yaw, v=st.v, omega=st.omega))
        # A6: lower bound is the matrix axis v_min (was hardwired 0.0, which
        # welded fwd-only into the replayer and made every v_min<0 cell a
        # false negative -- the reverse permission never reached the plant).
        cmd_v = min(max(out.v_cmd, args.v_min), args.v_max)
        cmd_w = max(min(out.omega_cmd, 2.0), -2.0)
        plant.step(cmd_v, cmd_w, Ts)
        e_y.append(abs(e))
        e_psi.append(abs(e_psi_v))
        dv.append((cmd_v - prev_cmd[0]) ** 2 + (cmd_w - prev_cmd[1]) ** 2)
        prev_cmd = np.array([cmd_v, cmd_w])

    e_y_arr = np.array(e_y) if e_y else np.array([0.0])
    path_len = float(traj.total_length)
    metrics = {
        "path_length_m": round(path_len, 3),
        "recorded_poses": len(poses),
        "completed": bool(reached),
        # A5.4 accepted-arc exports (raw vs accepted delta = projection lies)
        "raw_arc_final": round(raw_arc_final, 4),
        "accepted_arc_final": round(acc_arc_final, 4),
        "arc_high_watermark": round(high_watermark, 4),
        "progress_ratio": round(min(high_watermark / path_len, 1.0), 4) if path_len > 1e-9 else 0.0,
        "projection_jump_count": jump_episodes,
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
