#!/usr/bin/env python3
"""B6 chain step [2]: saved map (map_saver PGM/YAML) -> cleaning path.

Loads the map, derives candidate/known_free/obstacle/unknown masks, plans
the boustrophedon coverage path, and writes:
  <outdir>/cleaning_path.json   (b6_demo format, map frame)
  <outdir>/plan_stats.json      (planned coverage / waypoints / links)
  <outdir>/audit_masks.npz      (known_free/obstacle/unknown/candidate/
                                 task + res/origin/shift for the audit)
The path is RE-CENTRED so its first point sits at the tracking robot's
origin (tracking runs in its own odom frame); the inverse shift is stored
in audit_masks.npz ("shift") so the audit can map odom back to map frame.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

import os
_EXPLORER = os.environ.get("EXPLORER_REPO",
    str(Path(__file__).resolve().parents[3] / "ros2_tunnel_explorer"))
sys.path.insert(0, _EXPLORER)

from cleaning_mode.map_loader import load_map_grid  # noqa: E402
from cleaning_mode.obstacle_inflation import (  # noqa: E402
    apply_inflation_and_exclusions,
)
from cleaning_mode.boustrophedon import GridSpec  # noqa: E402
from cleaning_mode.path_smoother import smooth_path  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--footprint-radius-m", type=float, default=0.10)
    ap.add_argument("--lane-width-m", type=float, default=0.30)
    ap.add_argument("--speed", type=float, default=0.3)
    args = ap.parse_args()

    m = load_map_grid(args.map_yaml)
    known_free = m["known_free"]
    obstacle = m["obstacle"]
    unknown = m["unknown"]
    res = m["meta"]["resolution"]
    ox, oy = m["meta"]["origin"]
    candidate = known_free.copy()

    radius_cells = int(np.ceil(args.footprint_radius_m / res))
    blocked, executable = apply_inflation_and_exclusions(
        obstacle, unknown, known_free, candidate, radius_cells)

    # plan via the cleaning_mode planner over the four masks
    from cleaning_mode.coverage_planner import make_plan

    spec = GridSpec(res, ox, oy)
    cp = make_plan(candidate, known_free, obstacle, unknown, spec,
                   footprint_radius_m=args.footprint_radius_m,
                   lane_width_m=args.lane_width_m)
    planned = int(executable.sum())

    # world points (map frame) + speed profile via the smoother
    # make_plan returns map-frame waypoints directly
    world = [(p[0], p[1]) for p in cp.xy]
    speeds = [args.speed] * len(world)
    smoothed = smooth_path(world)

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # re-centre: tracking origin corresponds to the first path point
    sx, sy = smoothed[0][0], smoothed[0][1]
    rec = {"frame_id": "map",
           "poses": [[p[0] - sx, p[1] - sy, 0.0] for p in smoothed]}
    (outdir / "cleaning_path.json").write_text(json.dumps(rec, indent=1))

    stats = {
        "map": args.map_yaml,
        "known_free_cells": int(known_free.sum()),
        "executable_cells": planned,
        "path_waypoints": len(smoothed),
        "planned_coverage": cp.planned_coverage,
        "planner_reason": cp.reason,
        "path_length_m": round(sum(
            math.hypot(smoothed[i][0] - smoothed[i - 1][0],
                       smoothed[i][1] - smoothed[i - 1][1])
            for i in range(1, len(smoothed))), 3),
        "speed_mps": args.speed,
    }
    (outdir / "plan_stats.json").write_text(json.dumps(stats, indent=1))

    # audit masks in map frame + the shift (tracking origin in map frame)
    np.savez(outdir / "audit_masks.npz",
             known_free=known_free, obstacle=obstacle,
             unknown=unknown, candidate=candidate,
             executable=executable,
             res=res, ox=ox, oy=oy, shift_x=sx, shift_y=sy)
    print(json.dumps(stats, indent=1))


if __name__ == "__main__":
    main()
