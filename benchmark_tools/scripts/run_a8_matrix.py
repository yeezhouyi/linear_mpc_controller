#!/usr/bin/env python3
"""A8 v_min matrix runner (4060 doc A8.0-A8.3).

12 matrix cells = 3 geometries x 2 v_min x 2 controller_projection_mode,
audit side PINNED to the A5.2 accepted-arc auditor (A8.3: the only free
variable is controller_projection_mode).  Plus 3 replication rows
(global_legacy control + raw-arc audit) -- NOT matrix cells, physically
separated in the report; they reproduce the historical published numbers
with the lying ruler to show what those numbers lied about.

Every cell records the 12 doc-mandated fields.

Usage:
  python3 run_a8_matrix.py --geo-dir results/a8_replay/geo \
      --outdir results/a8_replay [--align-only]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.episode import run_tracking_episode
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import KinematicState, MpcParams, Trajectory

GEOS = ("cap", "nocap", "backward")
V_MINS = (0.0, -0.5)
MODES = (("stateful_windowed", "windowed"), ("global_legacy", "global"))

FIELDS = (
    "controller_projection_mode", "done_reason",
    "raw_arc_final", "accepted_arc_final",
    "progress_ratio", "skipped_arc_ratio", "max_arc_jump",
    "projection_jump_count", "projection_reacquire_count",
    "e_y_rms", "qp_failures", "v_end",
)


def load_traj(path: Path) -> Trajectory:
    d = json.loads(path.read_text())["traj"]
    return Trajectory(
        s=np.array(d["s"]), x=np.array(d["x"]), y=np.array(d["y"]),
        yaw=np.array(d["yaw"]), kappa=np.array(d["kappa"]),
        v=np.array(d["v"]), segment_gear=np.array(d["segment_gear"]))


def run_cell(traj, name, v_min, mode_doc, mode_code, audit_mode,
             max_steps, qp_max_iter=500):
    p = MpcParams(N=25, qp_max_iter=qp_max_iter, v_min=v_min,
                  controller_projection_mode=mode_code)
    controller = LinearMpcController(p, traj)
    controller.set_reference(traj)
    plant = DifferentialDrivePlant(
        x0=float(traj.x[0]), y0=float(traj.y[0]),
        yaw0=float(traj.yaw[0]), v0=0.0, omega0=0.0)
    res = run_tracking_episode(
        controller, plant, traj, track_name=name,
        max_steps=max_steps, audit_mode=audit_mode)
    return {
        "cell": name,
        "geometry": name.split("|")[0],
        "v_min": v_min,
        "controller_projection_mode": mode_doc,
        "audit_mode": audit_mode,
        "done_reason": res.done_reason,
        "raw_arc_final": round(float(res.arc_end), 3),
        "accepted_arc_final": round(float(res.arc_high_watermark), 3),
        "progress_ratio": round(float(res.progress_ratio), 4),
        "skipped_arc_ratio": round(float(res.skipped_arc_ratio), 4),
        "max_arc_jump": round(float(res.max_arc_jump), 3),
        "projection_jump_count": int(res.projection_jump_count),
        "projection_reacquire_count": int(res.projection_reacquire_count),
        "e_y_rms": round(float(res.e_y_rms), 4),
        "qp_failures": int(res.qp_failures),
        "v_end": round(float(res.v[-1]) if res.v else 0.0, 3),
        "steps": int(res.steps),
        "path_length_m": round(float(traj.s[-1]), 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--geo-dir", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--max-steps", type=int, default=8000)
    ap.add_argument("--align-only", action="store_true",
                    help="A8.0 stall-alignment check on the real plan only")
    args = ap.parse_args()

    geo_dir = Path(args.geo_dir)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # ---------- A8.0 stall-alignment check (real u9 plan) ----------
    real = load_traj(geo_dir / "a8_cap_real.json")
    align = run_cell(real, "cap_real|A8.0-alignment", 0.0,
                     "stateful_windowed", "windowed", "accepted",
                     args.max_steps)
    (outdir / "a8_0_alignment.json").write_text(json.dumps(align, indent=1))
    print("A8.0 alignment:", json.dumps(align, indent=1))
    if args.align_only:
        return

    # ---------- 12 matrix cells ----------
    cells = []
    for geo in GEOS:
        traj = load_traj(geo_dir / f"a8_{geo}.json")
        for v_min in V_MINS:
            for mode_doc, mode_code in MODES:
                name = f"{geo}|v_min={v_min}|{mode_doc}"
                print(f"=== cell {name} ===", flush=True)
                c = run_cell(traj, name, v_min, mode_doc, mode_code,
                             "accepted", args.max_steps)
                cells.append(c)
                print(json.dumps(c, indent=1), flush=True)
                (outdir / f"cell_{geo}_{v_min}_{mode_code}.json").write_text(
                    json.dumps(c, indent=1))

    # ---------- 3 replication rows (raw-arc ruler, global_legacy) ----------
    reps = []
    for geo in GEOS:
        traj = load_traj(geo_dir / f"a8_{geo}.json")
        name = f"{geo}|REPL|global_legacy|raw-audit"
        print(f"=== replication {name} ===", flush=True)
        r = run_cell(traj, name, 0.0, "global_legacy", "global",
                     "raw", args.max_steps)
        reps.append(r)
        print(json.dumps(r, indent=1), flush=True)
        (outdir / f"repl_{geo}.json").write_text(json.dumps(r, indent=1))

    (outdir / "matrix.json").write_text(json.dumps(
        {"fields": FIELDS, "cells": cells, "replication_rows": reps},
        indent=1))

    # ---------- markdown report ----------
    def row(c):
        return (f"| {c['geometry']} | {c['v_min']} "
                f"| {c['controller_projection_mode']} "
                f"| {c['done_reason']} | {c['raw_arc_final']} "
                f"| {c['accepted_arc_final']} | {c['progress_ratio']} "
                f"| {c['skipped_arc_ratio']} | {c['max_arc_jump']} "
                f"| {c['projection_jump_count']} "
                f"| {c['projection_reacquire_count']} | {c['e_y_rms']} "
                f"| {c['qp_failures']} | {c['v_end']} |")

    hdr = ("| geometry | v_min | controller_projection_mode | done_reason "
           "| raw_arc_final | accepted_arc_final | progress_ratio "
           "| skipped_arc_ratio | max_arc_jump | projection_jump_count "
           "| projection_reacquire_count | e_y_rms | qp_failures | v_end |\n"
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
    md = ["# A8 v_min matrix (accepted-arc audit, 12 cells)", "",
          f"A8.0 alignment (real u9 plan, stall target 7.3 m): "
          f"done={align['done_reason']}, "
          f"accepted_arc_final={align['accepted_arc_final']}, "
          f"raw_arc_final={align['raw_arc_final']}", "",
          hdr] + [row(c) for c in cells] + ["",
          "# Replication rows (global_legacy control + raw-arc audit) -- "
          "NOT matrix cells, physically separated per A8.3", "", hdr] \
          + [row(c) for c in reps] + [""]
    (outdir / "MATRIX.md").write_text("\n".join(md))
    print("matrix written:", outdir / "matrix.json")


if __name__ == "__main__":
    main()
