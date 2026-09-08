#!/usr/bin/env python3
"""One-shot triple measurement over tracked odom bags vs served-map masks
(review 2026-09-08, item 2).

For each bag reports:
  in_mask_frac      odom samples whose map-frame cell is inside executable
                    (gauge-frame sanity + the honest 'samples in plan region')
  swept_stained     swept area / stained area at footprint r=0.10
                    swept  = sum of per-sample disc areas (overlaps counted)
                    stained= unique visited cells * cell area (union)
                    (ratio-1 = overlap redundancy; reviewer probe ~16%)
  coverage_task@010 coverage_task with footprint_radius_m=0.10 (current gauge)
  coverage_task@015 coverage_task with footprint_radius_m=0.15 (half the
                    0.30 lane spacing -> seamless full-width strip)

Visited marking mirrors audit_b6_coverage.py exactly (same mapping, same
disc rule) so task@010 reproduces the committed audit numbers.

Usage:
  python3 audit_b6_triple.py --masks-npz <audit_masks.npz> \
      --bag run_v --bag run4 ... --outdir <dir>
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


def load_odom_xy(bag_dir: str):
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


def visited_mask(xy, sx, sy, ox, oy, res, h, w, radius):
    rr = int(math.ceil(radius / res))
    r2 = radius ** 2
    visited = np.zeros((h, w), dtype=bool)
    for (x, y) in xy:
        wx, wy = x + sx, y + sy
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
    return visited


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--masks-npz", required=True)
    ap.add_argument("--bag", action="append", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--r010", type=float, default=0.10)
    ap.add_argument("--r015", type=float, default=0.15)
    ap.add_argument("--sx", type=float, default=None,
                    help="override mask shift_x (default: value in npz)")
    ap.add_argument("--sy", type=float, default=None,
                    help="override mask shift_y (default: value in npz)")
    args = ap.parse_args()

    masks = np.load(args.masks_npz)
    known_free = masks["known_free"]
    executable = masks["executable"]
    res = float(masks["res"])
    ox, oy = float(masks["ox"]), float(masks["oy"])
    sx = args.sx if args.sx is not None else float(masks["shift_x"])
    sy = args.sy if args.sy is not None else float(masks["shift_y"])
    h, w = known_free.shape
    cell = res * res
    exec_sum = int(executable.sum())

    rows = []
    for bag in args.bag:
        xy = load_odom_xy(bag)
        n = len(xy)
        # in-mask fraction
        inside = 0
        for (x, y) in xy:
            wx, wy = x + sx, y + sy
            cx = int((wx - ox) / res)
            cy = int((wy - oy) / res)
            if 0 <= cy < h and 0 <= cx < w and executable[cy, cx]:
                inside += 1
        in_mask = inside / max(n, 1)
        # visited at both radii
        v10 = visited_mask(xy, sx, sy, ox, oy, res, h, w, args.r010)
        v15 = visited_mask(xy, sx, sy, ox, oy, res, h, w, args.r015)
        ve10 = int((v10 & executable).sum())
        ve15 = int((v15 & executable).sum())
        stained10 = int(v10.sum()) * cell
        vf10 = v10 & known_free
        vf15 = v15 & known_free
        rows.append({
            "bag": str(Path(bag).parent.name),
            "odom_samples": n,
            "shift_used": [round(sx, 3), round(sy, 3)],
            "in_mask_frac": round(in_mask, 4),
            "union_cells_r010": int(v10.sum()),
            "union_area_m2_r010": round(stained10, 3),
            "union_cells_r015": int(v15.sum()),
            "union_area_m2_r015": round(int(v15.sum()) * cell, 3),
            "gauge_credit_r010_over_r015_union": round(
                int(v10.sum()) / max(int(v15.sum()), 1), 4),
            "under_credit_pct_vs_r015": round(
                100.0 * (1.0 - int(v10.sum()) / max(int(v15.sum()), 1)), 2),
            "coverage_task_r010": round(ve10 / max(exec_sum, 1), 4),
            "visited_exec_r010": ve10,
            "coverage_task_r015": round(ve15 / max(exec_sum, 1), 4),
            "visited_exec_r015": ve15,
            "coverage_known_free_r010": round(
                int(vf10.sum()) / max(int(known_free.sum()), 1), 4),
            "coverage_known_free_r015": round(
                int(vf15.sum()) / max(int(known_free.sum()), 1), 4),
        })
        print(json.dumps(rows[-1], indent=1))

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "triple_measure.json").write_text(json.dumps(rows, indent=1))
    print("saved", outdir / "triple_measure.json")


if __name__ == "__main__":
    main()
