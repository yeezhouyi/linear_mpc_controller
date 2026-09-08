#!/usr/bin/env python3
"""A8 geometry generator (4060 doc A8.3).

Produces the matrix geometries as deterministic, dense (ds=0.05 m)
serpentines sharing one row layout (bounds from the real u9 rect plan,
lane width 0.30 m):

  a8_cap.json       row ends joined by half-circle caps (r = 0.15, u9 style)
  a8_nocap.json     row ends joined by straight 0.30 m jumps (pre-u9 style)
  a8_backward.json  first connector = reverse link: the robot backs out of
                    the row and reverses around a half circle into the next
                    row (segment_gear = -1, reference v < 0) -- a backward
                    A* connection per doc A9 semantics
  a8_cap_real.json  the REAL u9 plan converted as-is; NOT a matrix cell --
                    the A8.0 stall-alignment case only (target: the B6
                    first-run stall at 7.3 m)

Files carry the full Trajectory field set (x/y/yaw/kappa/s/v +
segment_gear) so the runner loads them WITHOUT re-deriving yaw.

Speed profile: repo-default complete_speed_curvature -- v = max(0.15,
min(0.5, 0.3/|kappa|)); reverse segments carry negative v (doc A9).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.types import Trajectory

DS = 0.05
LANE_W = 0.30
CAP_R = 0.15
V_FLOOR = 0.15
V_CAP = 0.5


def _speed_profile(kappa: np.ndarray, gear_seg: np.ndarray) -> np.ndarray:
    """Per-point speed; sign follows the point's outgoing segment gear."""
    v = np.array([max(V_FLOOR, min(V_CAP, 0.3 / max(abs(k), 1e-6)))
                  for k in kappa])
    sign = np.array([1.0 if (i < gear_seg.size and gear_seg[i] >= 0)
                     else -1.0 for i in range(kappa.size)])
    return v * sign


def _finish(x, y, gear_pts):
    """Build the Trajectory; gear_pts is per-POINT, reduced to per-segment
    by taking the outgoing segment's gear (the last point repeats)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = x.size
    s = np.zeros(n)
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    yaw = np.zeros(n)
    for i in range(n):
        j0, j1 = max(0, i - 1), min(n - 1, i + 1)
        yaw[i] = math.atan2(y[j1] - y[j0], x[j1] - x[j0])
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        arc = 0.5 * ((s[i] - s[i - 1]) + (s[i + 1] - s[i]))
        if arc > 1e-9:
            kappa[i] = ((yaw[i + 1] - yaw[i - 1] + math.pi)
                        % (2 * math.pi) - math.pi) / (2 * arc)
    gear_seg = np.asarray(gear_pts[:-1], dtype=float)   # outgoing segment
    # Reverse segments: reference yaw is the ROBOT HEADING (= tangent + pi),
    # mirroring the golden-cusp convention (yaw stays, gear flips) -- the
    # chord-derived yaw above is the tangent direction, so flip it by pi on
    # every point whose outgoing segment is reverse-driven.
    for i in range(n):
        g = gear_seg[i] if i < n - 1 else gear_seg[-1]
        if g < 0:
            yaw[i] = (yaw[i] + math.pi + math.pi) % (2 * math.pi) - math.pi
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=kappa,
                      v=_speed_profile(kappa, gear_seg),
                      segment_gear=gear_seg)


def _push(xs, ys, gs, pts, g):
    for p in pts:
        if xs and math.hypot(p[0] - xs[-1], p[1] - ys[-1]) < 1e-9:
            gs[-1] = g          # degenerate: update outgoing gear instead
            continue
        xs.append(p[0]); ys.append(p[1]); gs.append(g)


def _row(x0, x1, y):
    n = max(2, int(round(abs(x1 - x0) / DS)) + 1)
    return [(x0 + (x1 - x0) * i / (n - 1), y) for i in range(n)]


def _straight_conn(x, y0, y1):
    n = max(2, int(round(abs(y1 - y0) / DS)) + 1)
    return [(x, y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]


def _cap(x_edge, y0, y1, bulge):
    """Half-circle cap at a row edge, centre (x_edge, mid), radius CAP_R,
    traversed from y0 to y1 with heading rotating continuously."""
    yc = 0.5 * (y0 + y1)
    n = max(4, int(round((math.pi * CAP_R) / DS)))
    return [(x_edge + bulge * CAP_R * math.cos(-math.pi / 2
                                                + math.pi * i / n),
             yc + CAP_R * math.sin(-math.pi / 2 + math.pi * i / n))
            for i in range(n + 1)]


def serpentine(n_rows, x0, x1, y0, first_connector):
    """Boustrophedon: row 0 runs x0->x1 (+x); every subsequent connector per
    `first_connector` except the FIRST connector of the backward geometry,
    which is the reverse link.  Rows alternate direction; connectors sit at
    alternating row edges."""
    xs, ys, gs = [], [], []
    y = y0
    _push(xs, ys, gs, _row(x0, x1, y), +1.0)   # row 0: x0 -> x1
    edge = x1
    for k in range(1, n_rows):
        y_next = y0 + k * LANE_W
        if k == 1 and first_connector == "reverse_link":
            # -- backward A* link (doc A9): reverse out, reverse around the
            # half circle, arrive on the next row heading-flipped.
            x_back = edge - 4.0 * CAP_R
            _push(xs, ys, gs, _row(edge, x_back, y), -1.0)          # back out
            cap_pts = _cap(x_back, y, y_next, bulge=+1.0)
            cap_pts = [(2 * x_back - px, py) for (px, py) in cap_pts]
            _push(xs, ys, gs, cap_pts, -1.0)                        # reverse arc
            _push(xs, ys, gs, _row(x_back, edge, y_next), -1.0)     # reverse in
        elif first_connector == "cap":
            _push(xs, ys, gs,
                  _cap(edge, y, y_next, bulge=+1.0 if edge == x1 else -1.0),
                  +1.0)
        else:
            _push(xs, ys, gs, _straight_conn(edge, y, y_next), +1.0)
        # next row runs opposite; connector was at the current edge
        y = y_next
        new_edge = x0 if edge == x1 else x1
        _push(xs, ys, gs, _row(edge, new_edge, y), +1.0)
        edge = new_edge
    return xs, ys, gs


def real_plan_traj(poses):
    x = [float(p[0]) for p in poses]
    y = [float(p[1]) for p in poses]
    return _finish(x, y, [+1.0] * len(x))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real-plan", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--row-len", type=float, default=5.0)
    args = ap.parse_args()

    rec = json.loads(Path(args.real_plan).read_text())
    pts = [(float(p[0]), float(p[1])) for p in rec["poses"]]
    x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
    y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    def emit(name, traj, meta):
        payload = {
            "frame_id": "map",
            "meta": meta,
            "traj": {
                "x": [round(float(v), 6) for v in traj.x],
                "y": [round(float(v), 6) for v in traj.y],
                "yaw": [round(float(v), 6) for v in traj.yaw],
                "kappa": [round(float(v), 6) for v in traj.kappa],
                "s": [round(float(v), 6) for v in traj.s],
                "v": [round(float(v), 6) for v in traj.v],
                "segment_gear": [float(v) for v in traj.segment_gear],
            },
        }
        p = out / name
        p.write_text(json.dumps(payload))
        print(f"{name}: {len(traj.x)} pts, L={traj.s[-1]:.2f} m")

    emit("a8_cap_real.json", real_plan_traj(rec["poses"]),
         {"kind": "cap_real", "source": str(args.real_plan)})

    xr0, xr1 = x0 + 0.1, x0 + 0.1 + args.row_len
    yr0 = y0 + 0.1
    for kind in ("cap", "nocap", "backward"):
        conn = {"cap": "cap", "nocap": "straight",
                "backward": "reverse_link"}[kind]
        xs_l, ys_l, gs_l = serpentine(args.rows, xr0, xr1, yr0, conn)
        t = _finish(xs_l, ys_l, gs_l)
        emit(f"a8_{kind}.json", t,
             {"kind": kind, "rows": args.rows, "row_len": args.row_len,
              "lane_w": LANE_W, "connector": conn})


if __name__ == "__main__":
    main()
