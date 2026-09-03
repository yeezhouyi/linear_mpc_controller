"""Reference trajectory generators & helpers (plan U2/U6: 4 track classes).

Tracks are built from curvature segments integrated exactly, then sampled at
fixed arc spacing ``ds``.  Velocities can be constant per segment or globally
scaled; curvature is recorded exactly and can be cross-checked against the
numeric estimator in :mod:`trajectory_tools.curvature_estimator`.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence

import numpy as np

from mpc_core.types import Trajectory, TrackPointKind, wrap_angle


def integrate_segments(segments: Sequence[dict], ds: float = 0.02) -> Trajectory:
    """Integrate a curvature-segment description into a dense Trajectory.

    Segment dict keys: ``kind`` in {'straight','arc'}, ``length`` (m),
    ``kappa`` (1/m, sign = turn direction; unused for straight),
    ``v`` (m/s reference speed).

    Integration formulas (exact for constant curvature):
      arc:      dx = (sin(y0 + k*l) - sin(y0))/k, dy = (cos(y0) - cos(y0 + k*l))/k
      straight: dx = cos(y0)*l, dy = sin(y0)*l
    """
    # Seed the origin vertex with the *first* segment's curvature/speed so the
    # trajectory is consistent at s = 0 (used as a prediction-window base).
    first = segments[0]
    first_k = float(first.get("kappa", 0.0)) if first["kind"] == "arc" else 0.0
    first_v = float(first.get("v", 0.5))
    pts_s: List[float] = [0.0]
    pts_x: List[float] = [0.0]
    pts_y: List[float] = [0.0]
    pts_yaw: List[float] = [0.0]
    pts_k: List[float] = [first_k]
    pts_v: List[float] = [first_v]

    yaw = 0.0
    s = 0.0
    x = y = 0.0
    prev_k = first_k
    prev_v = first_v

    for seg in segments:
        kind = seg["kind"]
        length = float(seg["length"])
        k = float(seg.get("kappa", 0.0))
        v = float(seg.get("v", prev_v))
        if kind == "arc" and abs(k) < 1e-12:
            kind = "straight"
        n = max(2, int(math.ceil(length / ds)))
        dl = length / n
        for _ in range(n):
            if kind == "arc":
                if abs(k) > 1e-12:
                    r = 1.0 / k
                    x_new = x + (math.sin(yaw + k * dl) - math.sin(yaw)) / k
                    y_new = y + (math.cos(yaw) - math.cos(yaw + k * dl)) / k
                else:
                    x_new = x + math.cos(yaw) * dl
                    y_new = y + math.sin(yaw) * dl
            else:
                x_new = x + math.cos(yaw) * dl
                y_new = y + math.sin(yaw) * dl
            yaw = wrap_angle(yaw + k * dl)
            x, y = x_new, y_new
            s += dl
            pts_s.append(s)
            pts_x.append(x)
            pts_y.append(y)
            pts_yaw.append(yaw)
            pts_k.append(k)
            pts_v.append(v)
        prev_k = k
        prev_v = v

    return Trajectory(
        s=np.array(pts_s),
        x=np.array(pts_x),
        y=np.array(pts_y),
        yaw=np.array(pts_yaw),
        kappa=np.array(pts_k),
        v=np.array(pts_v),
        kind=TrackPointKind.WITH_VELOCITY_CURVATURE,
    )


def make_straight(length: float = 8.0, v: float = 0.8, ds: float = 0.02) -> Trajectory:
    return integrate_segments([{"kind": "straight", "length": length, "v": v}], ds=ds)


def make_circle(radius: float = 2.0, v: float = 0.6, ds: float = 0.02) -> Trajectory:
    """Full CCW circle (positive curvature)."""
    return integrate_segments(
        [{"kind": "arc", "length": 2.0 * math.pi * radius, "kappa": 1.0 / radius, "v": v}],
        ds=ds,
    )


def make_s_curve(straight1: float = 1.5, radius: float = 1.5, straight2: float = 1.5,
                 v: float = 0.6, ds: float = 0.02) -> Trajectory:
    """S shape: straight -> left arc (90 deg) -> straight -> right arc -> straight."""
    quarter = math.pi * radius / 2.0
    segs = [
        {"kind": "straight", "length": straight1, "v": v},
        {"kind": "arc", "length": quarter, "kappa": 1.0 / radius, "v": v},
        {"kind": "straight", "length": straight2, "v": v},
        {"kind": "arc", "length": quarter, "kappa": -1.0 / radius, "v": v},
        {"kind": "straight", "length": straight1, "v": v},
    ]
    return integrate_segments(segs, ds=ds)


def make_u_turn(approach: float = 1.0, radius: float = 1.2, exit_l: float = 1.0,
                v: float = 0.5, ds: float = 0.02) -> Trajectory:
    """U turn: straight -> 180 deg arc (half circle) -> straight."""
    segs = [
        {"kind": "straight", "length": approach, "v": v},
        {"kind": "arc", "length": math.pi * radius, "kappa": 1.0 / radius, "v": v},
        {"kind": "straight", "length": exit_l, "v": v},
    ]
    return integrate_segments(segs, ds=ds)


def generate_benchmark_tracks(**overrides) -> Dict[str, Trajectory]:
    """The four formal benchmark tracks (AE1 / R21)."""
    kw = dict(
        straight=dict(length=overrides.get("straight_len", 8.0), v=overrides.get("straight_v", 0.8)),
        circle=dict(radius=overrides.get("circle_r", 2.0), v=overrides.get("circle_v", 0.6)),
        s_curve=dict(v=overrides.get("s_v", 0.6)),
        u_turn=dict(v=overrides.get("u_v", 0.5)),
    )
    return {
        "straight": make_straight(**kw["straight"]),
        "circle": make_circle(**kw["circle"]),
        "s_curve": make_s_curve(**kw["s_curve"]),
        "u_turn": make_u_turn(**kw["u_turn"]),
    }
