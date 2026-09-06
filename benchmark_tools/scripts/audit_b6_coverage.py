#!/usr/bin/env python3
"""B6 chain step [4]: dual-denominator coverage audit of the MPC tracking
execution against the saved-map masks (plan_from_map.py output).

Inputs: tracking odom bag + audit_masks.npz + plan_stats.json.
Visited = tool disc (footprint radius) along the tracked odom polyline,
mapped back into the map frame via the stored shift.

Denominators (R10, both reported, never swapped):
  coverage_task       visited∩executable / executable   (the planned region)
  coverage_known_free visited∩known_free  / known_free  (the whole known map)
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def load_odom_xy(bag_dir: str) -> list:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"),
                ConverterOptions("", ""))
    xy = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == "/odom":
            m = deserialize_message(data, Odometry)
            xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
    return xy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track-bag", required=True)
    ap.add_argument("--masks-npz", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--footprint-radius-m", type=float, default=0.10)
    args = ap.parse_args()

    masks = np.load(args.masks_npz)
    known_free = masks["known_free"]
    executable = masks["executable"]
    res = float(masks["res"])
    ox, oy = float(masks["ox"]), float(masks["oy"])
    sx, sy = float(masks["shift_x"]), float(masks["shift_y"])
    h, w = known_free.shape

    xy = load_odom_xy(args.track_bag)
    rr = int(math.ceil(args.footprint_radius_m / res))
    r2 = args.footprint_radius_m ** 2
    visited = np.zeros_like(known_free)
    driven = 0.0
    prev = None
    for (x, y) in xy:
        wx, wy = x + sx, y + sy  # tracking origin -> map frame
        if prev is not None:
            driven += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    px = ox + (nx + 0.5) * res
                    py = oy + (ny + 0.5) * res
                    if (px - wx) ** 2 + (py - wy) ** 2 <= r2:
                        visited[ny, nx] = True

    vf = visited & known_free
    ve = visited & executable
    plan_stats = json.loads(
        (Path(args.masks_npz).parent / "plan_stats.json").read_text())

    out = {
        "odom_samples": len(xy),
        "driven_length_m": round(driven, 3),
        "planned_path_length_m": plan_stats["path_length_m"],
        "execution_overhead_ratio": round(
            driven / max(plan_stats["path_length_m"], 1e-9), 3),
        "known_free_cells": int(known_free.sum()),
        "executable_cells": int(executable.sum()),
        "visited_executable_cells": int(ve.sum()),
        "coverage_task": round(int(ve.sum()) / max(int(executable.sum()), 1), 4),
        "coverage_known_free": round(int(vf.sum()) / max(int(known_free.sum()), 1), 4),
        "footprint_radius_m": args.footprint_radius_m,
    }
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    np.savez(outdir / "audit_visited.npz", visited=visited)
    (outdir / "coverage_audit.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
