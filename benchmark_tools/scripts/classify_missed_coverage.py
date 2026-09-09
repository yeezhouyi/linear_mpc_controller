#!/usr/bin/env python3
"""Missed-coverage classification + overlay visualisation (tunnel P0).

For each chain run (odom bag): mark cells reached by the PLANNED path and
by the ACTUAL trajectory at the audit gauge radii (r010 / r015), then
classify every executable cell:

  plan_miss      executable, NOT swept by the planned path  (planning-stage
                 omission)
  exec_miss      planned AND executable, but not swept by the actual
                 trajectory (execution deviation / connector failure)
  visited_outside executable=false but visited (localisation / projection
                 statistical offset)
  unreachable    candidate but not executable (scene itself)

Outputs per run: overlay PNG (executable grey, obstacles black, planned
sweep blue, actual trajectory red, exec_miss orange, plan_miss purple)
and a classification JSON; plus an aggregate markdown table.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from benchmark_tools.scripts.audit_b6_triple import load_odom_xy  # noqa: E402


def dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    h, w = mask.shape
    cov = np.zeros((h, w), dtype=bool)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dy * dy + dx * dx > radius_cells * radius_cells:
                continue
            sy = slice(max(0, -dy), h - max(0, dy))
            sx = slice(max(0, -dx), w - max(0, dx))
            ty = slice(max(0, dy), h - max(0, -dy))
            tx = slice(max(0, dx), w - max(0, -dx))
            cov[ty, tx] |= mask[sy, sx]
    return cov


def to_cells(xy, ox, oy, res, w, h):
    cells = set()
    for x, y in xy:
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if 0 <= cx < w and 0 <= cy < h:
            cells.add((cy, cx))
    return cells


def cells_to_mask(cells, shape):
    m = np.zeros(shape, dtype=bool)
    for (cy, cx) in cells:
        m[cy, cx] = True
    return m


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--masks-npz", required=True)
    ap.add_argument("--plan", required=True)
    ap.add_argument("--bag", action="append", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--r010", type=float, default=0.10)
    ap.add_argument("--r015", type=float, default=0.15)
    ap.add_argument("--sx", type=float, default=0.0,
                    help="physical shift applied to the ODOMETRY only "
                         "(canonical gauge: 0,0)")
    ap.add_argument("--sy", type=float, default=0.0)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.load(args.masks_npz)
    res = float(d["res"])
    ox = float(d["ox"])
    oy = float(d["oy"])
    known_free = d["known_free"]
    obstacle = d["obstacle"]
    candidate = d["candidate"]
    executable = d["executable"]
    h, w = executable.shape
    n_exec = int(executable.sum())

    raw_plan = [(float(p[0]), float(p[1])) for p in
                json.loads(Path(args.plan).read_text())["poses"]]
    # plan frame = map frame; mask frame is shifted by the npz shift.
    sx_plan = float(d["shift_x"])
    sy_plan = float(d["shift_y"])
    plan_pts = []
    for k in range(len(raw_plan)):
        x0, y0 = raw_plan[k]
        if k + 1 < len(raw_plan):
            x1, y1 = raw_plan[k + 1]
        else:
            x1, y1 = x0, y0
        seglen = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(round(seglen / (0.5 * res))) + 1)
        for j in range(n):
            u = j / (n - 1) if k + 1 < len(raw_plan) else 0.0
            plan_pts.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u))
    plan_pts = [(x + sx_plan, y + sy_plan) for (x, y) in plan_pts]

    agg = {}
    Path(args.outdir).mkdir(parents=True, exist_ok=True)
    for bag in args.bag:
        name = Path(bag).parent.name + "_" + Path(bag).name
        odom = [(x + args.sx, y + args.sy) for (x, y) in load_odom_xy(bag)]
        plan_cells = to_cells(plan_pts, ox, oy, res, w, h)
        odom_cells = to_cells(odom, ox, oy, res, w, h)
        out = {}
        for r, tag in ((args.r010, "r010"), (args.r015, "r015")):
            rc = max(1, int(round(r / res)))
            planned = dilate(cells_to_mask(plan_cells, (h, w)), rc)
            visited = dilate(cells_to_mask(odom_cells, (h, w)), rc)
            plan_miss = executable & ~planned
            exec_miss = executable & planned & ~visited
            visited_outside = (~executable) & visited
            unreachable = candidate & ~executable
            out[tag] = dict(
                plan_miss=int(plan_miss.sum()),
                exec_miss=int(exec_miss.sum()),
                visited_outside=int(visited_outside.sum()),
                unreachable=int(unreachable.sum()),
                plan_covered_frac=round(
                    float((executable & planned).sum() / n_exec), 4),
                visited_frac=round(
                    float((executable & visited).sum() / n_exec), 4),
            )
            if tag == "r015":
                pm, em = plan_miss, exec_miss
        # ---- overlay PNG (r015 view) ----
        img = np.zeros((h, w, 3), dtype=float)
        img[known_free] = (0.92, 0.92, 0.92)
        img[executable] = (0.75, 0.85, 0.75)
        img[obstacle] = (0.1, 0.1, 0.1)
        img[~candidate & ~obstacle] = (0.85, 0.85, 1.0)
        img[pm] = (0.65, 0.3, 0.85)      # planning miss: purple
        img[em] = (1.0, 0.55, 0.1)       # execution miss: orange
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(img, origin="lower",
                  extent=[ox, ox + w * res, oy, oy + h * res])
        px = [p[0] for p in plan_pts]
        py = [p[1] for p in plan_pts]
        ax.plot(px, py, "-", color="tab:blue", lw=1.2,
                label="planned path")
        if odom:
            ax.plot([p[0] for p in odom[::20]], [p[1] for p in odom[::20]],
                    "-", color="tab:red", lw=0.8, alpha=0.8,
                    label="actual trajectory")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("%s  (r015)  plan_miss=%d  exec_miss=%d"
                     % (name, out["r015"]["plan_miss"],
                        out["r015"]["exec_miss"]), fontsize=10)
        ax.set_aspect("equal")
        fig.savefig(Path(args.outdir) / (name + "_overlay.png"), dpi=150,
                    bbox_inches="tight")
        plt.close(fig)
        agg[name] = out
        print(name, json.dumps(out), flush=True)

    (Path(args.outdir) / "miss_classification.json").write_text(
        json.dumps(agg, indent=1))
    md = ["# Missed-coverage classification (r015 gauge)", "",
          "| run | plan_miss | exec_miss | visited_outside | unreachable"
          " | plan_covered_frac | visited_frac |",
          "|---|---|---|---|---|---|---|"]
    for name, o in agg.items():
        r = o["r015"]
        md.append("| %s | %d | %d | %d | %d | %.4f | %.4f |"
                  % (name, r["plan_miss"], r["exec_miss"],
                     r["visited_outside"], r["unreachable"],
                     r["plan_covered_frac"], r["visited_frac"]))
    (Path(args.outdir) / "miss_classification.md").write_text(
        "\n".join(md) + "\n")
    print("written:", args.outdir)


if __name__ == "__main__":
    main()
