#!/usr/bin/env python3
"""Projection-sequence golden (doc A7.2 item 5), Python side.

Fixed pose sequences over three reference geometries -- straight(+jump),
antiparallel fold (折返) and cusp reverse (尖点/后向连接) -- are projected
with the stateful chain (closest_point windowed + ProgressAllowanceGate for
the accepted arc, s_prev fed back like the controller does).  Every step
records (seg, w, e_y, raw_arc, accepted_arc, stage) plus the projection
parameter defaults in the file header.  The committed golden file freezes
current behaviour: any future drift of projection outputs or of the window /
gate parameter defaults trips the paired test (test_projection_golden.py).

Usage:
  python3 mpc_core/tools/generate_projection_golden.py   # (re)write the file
"""
from __future__ import annotations

import inspect
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.frenet import closest_point
from mpc_core.progress_gate import ProgressAllowanceGate
from mpc_core.types import MpcParams, Trajectory

GOLDEN_PATH = Path(__file__).resolve().parents[1] / "tests" / "data" / "projection_golden.json"

FWD_M = 0.30
BACK_M = 0.08
REACQUIRE_M = 1.00
WIDE_M = 5.00
HEADING_GATE_RAD = 1.5708
TIE_EPS_M = 0.05
CAPTURE_M = 0.30


# ------------------------------------------------------------ geometries
def straight_traj(length=6.0, ds=0.02):
    n = int(round(length / ds)) + 1
    s = np.linspace(0.0, length, n)
    return Trajectory(s=s, x=s.copy(), y=np.zeros(n), yaw=np.zeros(n),
                      kappa=np.zeros(n), v=np.full(n, 0.6))


def fold_traj(length=2.0, offset=0.30, n=101):
    """Out along +x at y=0, back along -x at y=offset (arc keeps rising)."""
    fx = np.linspace(0.0, length, n)
    rx = np.linspace(length, 0.0, n)
    x = np.concatenate([fx, rx])
    y = np.concatenate([np.zeros(n), np.full(n, offset)])
    yaw = np.concatenate([np.zeros(n), np.full(n, math.pi)])
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=np.zeros(x.size),
                      v=np.full(x.size, 0.5))


def cusp_traj(n=101):
    """Forward (+1) to x=2, zero-speed cusp, reverse (-1) back to x=0."""
    fx = np.linspace(0.0, 2.0, n)
    rx = np.linspace(2.0, 0.0, n)
    x = np.concatenate([fx, rx])
    y = np.zeros(x.size)
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + abs(x[i] - x[i - 1])
    v = np.zeros(x.size)
    v[: n - 1] = 0.8
    v[n:] = -0.5
    gear = np.concatenate([np.ones(n - 1), np.full(n, -1.0)])
    return Trajectory(s=s, x=x, y=y, yaw=np.zeros(x.size),
                      kappa=np.zeros(x.size), v=v, segment_gear=gear)


def _seq_straight_jump():
    """Track along +x, then a 3 m pose teleport mid-run (accepted-arc freeze)."""
    poses = [(0.0 + 0.05 * k, 0.05, 0.0) for k in range(6)]
    poses += [(3.0, 0.05, 0.0)]                 # teleport
    poses += [(3.05 + 0.05 * k, 0.05, 0.0) for k in range(4)]
    return poses


def _seq_fold():
    """Drive out on lane A, then cross to the middle (equidistant, stage 3),
    then park on lane B."""
    poses = [(0.4, 0.0, 0.0), (1.0, 0.0, 0.0), (1.5, 0.0, 0.0),
             (1.8, 0.15, 0.0),                  # between lanes -> ambiguous
             (1.8, 0.30, 0.0), (1.2, 0.30, math.pi), (0.6, 0.30, math.pi)]
    return poses


def _seq_cusp():
    """Approach the cusp forward, then drive away in reverse (body +x)."""
    poses = [(1.2, 0.0, 0.0), (1.6, 0.0, 0.0), (1.9, 0.0, 0.0),
             (2.05, 0.0, 0.0),                  # just past the cusp (dup vertex)
             (1.9, 0.0, 0.0), (1.5, 0.0, 0.0), (1.0, 0.0, 0.0)]
    return poses


SEQUENCES = {
    "straight_jump": (straight_traj, _seq_straight_jump),
    "fold": (fold_traj, _seq_fold),
    "cusp": (cusp_traj, _seq_cusp),
}


# ------------------------------------------------------------ runner
def generate_sequence(seq_name: str):
    """Return list of per-step records for one sequence (deterministic)."""
    traj_fn, poses_fn = SEQUENCES[seq_name]
    traj = traj_fn()
    gate = ProgressAllowanceGate(v_max=MpcParams().v_max, Ts=MpcParams().Ts)
    records = []
    s_prev = None
    first = True
    for (px, py, yaw) in poses_fn():
        if first:                       # prime the gate with the start pose
            _, _, _, a0, _ = closest_point(traj, px, py)
            gate = ProgressAllowanceGate(v_max=MpcParams().v_max,
                                         Ts=MpcParams().Ts,
                                         accepted_arc=float(a0))
            gate.step(traj, px, py, float(a0))
            first = False
        seg, w, e_y, arc, stage = closest_point(
            traj, px, py, yaw=yaw, s_prev=s_prev)
        dec = gate.step(traj, px, py, arc)
        ap = traj.sample_by_s(arc) if stage != 3 else None
        near = (math.hypot(px - ap.x, py - ap.y) <= CAPTURE_M
                if ap is not None else False)
        if not dec.accepted and near:
            gate.rebaseline(arc)
        s_prev = arc if stage != 3 else s_prev
        records.append({
            "pose": [round(float(px), 6), round(float(py), 6),
                     round(float(yaw), 6)],
            "seg": int(seg), "w": round(float(w), 9),
            "e_y": None if math.isnan(e_y) else round(float(e_y), 9),
            "raw_arc": round(float(arc), 9),
            "accepted_arc": round(float(gate.accepted_arc), 9),
            "stage": int(stage),
        })
    return traj, records


def defaults_header():
    """Window/gate parameter defaults that must never silently drift."""
    sig = inspect.signature(closest_point)
    p = MpcParams()
    return {
        "back_m": sig.parameters["back_m"].default,
        "fwd_m": sig.parameters["fwd_m"].default,
        "reacquire_m": sig.parameters["reacquire_m"].default,
        "wide_m": sig.parameters["wide_m"].default,
        "heading_gate_rad": sig.parameters["heading_gate_rad"].default,
        "tie_eps_m": sig.parameters["tie_eps_m"].default,
        "projection_margin_m": p.projection_margin_m,
        "allowance_cap_m": p.allowance_cap_m,
        "odom_step_noise_m": p.odom_step_noise_m,
    }


def build_golden():
    seqs = {}
    for name in SEQUENCES:
        _, records = generate_sequence(name)
        seqs[name] = records
    return {"params": defaults_header(), "sequences": seqs}


def main():
    GOLDEN_PATH.parent.mkdir(parents=True, exist_ok=True)
    GOLDEN_PATH.write_text(json.dumps(build_golden(), indent=1))
    print(f"golden written: {GOLDEN_PATH} ({GOLDEN_PATH.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
