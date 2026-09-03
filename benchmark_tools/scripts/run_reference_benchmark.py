#!/usr/bin/env python3
"""Reference-core benchmark runner (offline, deterministic).

Runs pure linear MPC on the four formal tracks and writes a run manifest +
aggregated metrics.  This is the *reference core* gate (R21/R22/R23); the
formal 5-run Gazebo protocol is executed later with the C++/ROS2 stack
(see docs/benchmark_protocol.md).

Usage:
    python benchmark_tools/scripts/run_reference_benchmark.py \
        --runs 1 --seed 0 --outdir outputs/bench_ref
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from benchmark_tools.compute_tracking_metrics import (  # noqa: E402
    compute_tracking_metrics,
    format_metrics_md,
)
from mpc_core.episode import run_tracking_episode  # noqa: E402
from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from mpc_core.mpc import LinearMpcController  # noqa: E402
from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import (  # noqa: E402
    generate_benchmark_tracks,
)

# Deterministic per-track initial conditions (R21: same IC for all controllers).
TRACK_ICS = {
    "straight": dict(x0=0.5, y0=0.35, yaw0=0.15, v0=0.0, omega0=0.0),
    "circle": dict(x0=0.0, y0=-0.25, yaw0=0.12, v0=0.3, omega0=0.0),
    "s_curve": dict(x0=-0.3, y0=0.25, yaw0=0.15, v0=0.0, omega0=0.0),
    "u_turn": dict(x0=-0.3, y0=0.25, yaw0=0.15, v0=0.0, omega0=0.0),
}

MPC_CFG = dict(
    Ts=0.05,
    N=25,
    Q_diag=(120.0, 25.0, 3.0, 1.5),
    Q_F_diag=(200.0, 40.0, 5.0, 3.0),
    S_diag=(0.4, 0.4),
    v_min=0.0,
    v_max=1.5,
    omega_max=2.0,
    a_max=1.0,
    alpha_max=2.0,
    lookahead_m=0.0,
)


def git_head() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True, cwd=os.getcwd(),
        )
        return out.stdout.strip()
    except Exception:
        return "unknown"


def run_one(track_name, traj, ic, params, rng, max_steps=6000):
    ctrl = LinearMpcController(params, traj)
    # small deterministic initial jitter per run seed (kept for reproducibility)
    plant = DifferentialDrivePlant(
        x0=ic["x0"], y0=ic["y0"], yaw0=ic["yaw0"], v0=ic["v0"], omega0=ic["omega0"], seed=None
    )
    res = run_tracking_episode(ctrl, plant, traj, track_name, max_steps=max_steps)
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--outdir", default="outputs/bench_ref")
    args = ap.parse_args()

    tracks = generate_benchmark_tracks()
    params = MpcParams(**MPC_CFG)
    cfg_hash = params.config_hash()
    commit = git_head()

    outdir = args.outdir
    os.makedirs(outdir, exist_ok=True)

    all_metrics = []
    for track_name, traj in tracks.items():
        ic = TRACK_ICS[track_name]
        for run_i in range(args.runs):
            rng = np.random.default_rng(args.seed * 100 + run_i)
            res = run_one(track_name, traj, ic, params, rng)
            m = compute_tracking_metrics(res, v_ref=float(np.median(traj.v)))
            m["run"] = run_i
            m["seed"] = int(rng.integers(0, 2**31 - 1))
            all_metrics.append(m)
            print(format_metrics_md(m))

    manifest = {
        "kind": "reference-core benchmark",
        "controller": "linear_mpc (mpc_core reference implementation)",
        "config_hash": cfg_hash,
        "mpc_params": {k: (list(v) if isinstance(v, tuple) else v) for k, v in MPC_CFG.items()},
        "commit": commit,
        "seed_base": args.seed,
        "runs_per_track": args.runs,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "metrics": all_metrics,
    }
    with open(os.path.join(outdir, "run_manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    md_lines = [
        "# Reference-core benchmark (pure linear MPC, offline)",
        "",
        f"- commit: `{commit}`  config hash: `{cfg_hash}`  runs/track: {args.runs}",
        "",
        "| track | done | steps | e_y_rms | e_y_p95 | e_y_max | e_psi_rms | e_v_rms | qp_mean(us) | qp_p95(us) | qp_fail | fallback | viol |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    md_lines += [format_metrics_md(m) for m in all_metrics]

    # per-track aggregation across runs (R23: median + p95 over the runs)
    if args.runs > 1:
        md_lines += ["", "### Per-track aggregation over runs", "",
                     "| track | e_y_rms med | e_y_rms p95(runs) | e_y_p95 med | qp_fail total | completed |",
                     "|---|---|---|---|---|---|"]
        for track_name in tracks:
            ms = [m for m in all_metrics if m["track"] == track_name]
            ey = np.array([m["e_y_rms"] for m in ms])
            p95 = np.array([m["e_y_p95"] for m in ms])
            qpf = sum(m["qp_failures"] for m in ms)
            done = sum(1 for m in ms if m["completed"])
            md_lines.append(
                f"| {track_name} | {np.median(ey):.4f} | {np.percentile(ey, 95):.4f} | "
                f"{np.median(p95):.4f} | {qpf} | {done}/{len(ms)} |")
        manifest["aggregates"] = {
            t: {
                "e_y_rms_median": float(np.median([m["e_y_rms"] for m in all_metrics if m["track"] == t])),
                "e_y_rms_p95_across_runs": float(np.percentile(
                    [m["e_y_rms"] for m in all_metrics if m["track"] == t], 95)),
                "completed": sum(1 for m in all_metrics if m["track"] == t and m["completed"]),
            }
            for t in tracks
        }
        with open(os.path.join(outdir, "run_manifest.json"), "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2, ensure_ascii=False)

    md_path = os.path.join(outdir, "benchmark_results.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md_lines) + "\n")
    print(f"\nwrote {md_path} and {os.path.join(outdir, 'run_manifest.json')}")


if __name__ == "__main__":
    main()
