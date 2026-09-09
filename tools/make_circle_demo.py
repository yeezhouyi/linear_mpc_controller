#!/usr/bin/env python3
"""Generate the README demo clip for the reference-core closed loop.

Re-runs the deterministic circle track through the real linear-MPC episode
(reference core, numpy ADMM) and renders an animated playback of the closed
loop: reference path, executed trajectory (rebuilt from the recorded
v/omega by the same kinematic integrator used inside the episode), and the
per-step lateral error.

Outputs (into results/demo_20260909/):
  circle_closed_loop.json  -- executed trajectory + metrics (evidence)
  circle_closed_loop.gif   -- playback animation

Honest labeling: this is the OFFLINE reference core (Python/numpy ADMM,
perfect velocity tracking, no noise), not the C++/ROS2 Gazebo stack.  The
Gazebo numbers live in results/mpc_smoke_20260909/.

Usage: python3 tools/make_circle_demo.py [--frames 120] [--out results/demo_20260909]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import subprocess
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from benchmark_tools.scripts.run_reference_benchmark import (  # noqa: E402
    MPC_CFG,
    TRACK_ICS,
)
from mpc_core.episode import run_tracking_episode  # noqa: E402
from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from mpc_core.mpc import LinearMpcController  # noqa: E402
from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import generate_benchmark_tracks  # noqa: E402

TS = 0.05
TRACK = "circle"


def git_head() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True,
            text=True, check=True, cwd=ROOT).stdout.strip()
    except Exception:
        return "unknown"


def run_closed_loop() -> tuple:
    tracks = generate_benchmark_tracks()
    traj = tracks[TRACK]
    ic = TRACK_ICS[TRACK]
    ctrl = LinearMpcController(MpcParams(**MPC_CFG), traj)
    plant = DifferentialDrivePlant(
        x0=ic["x0"], y0=ic["y0"], yaw0=ic["yaw0"],
        v0=ic["v0"], omega0=ic["omega0"], seed=None)
    res = run_tracking_episode(ctrl, plant, traj, TRACK, max_steps=6000)
    return traj, ic, res


def rebuild_xy(ic: dict, v_seq, omega_seq) -> tuple:
    """Same kinematic integrator as DifferentialDrivePlant.step (lag=0)."""
    x, y, yaw = float(ic["x0"]), float(ic["y0"]), float(ic["yaw0"])
    xs, ys = [x], [y]
    for v, w in zip(v_seq, omega_seq):
        yaw = yaw + float(w) * TS
        x = x + float(v) * math.cos(yaw) * TS
        y = y + float(v) * math.sin(yaw) * TS
        xs.append(x)
        ys.append(y)
    return xs, ys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=130)
    ap.add_argument("--out", default=str(ROOT / "results" / "demo_20260909"))
    ap.add_argument("--skip-render", action="store_true")
    args = ap.parse_args()

    outdir = pathlib.Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    traj, ic, res = run_closed_loop()
    xs, ys = rebuild_xy(ic, res.v, res.omega)
    n = len(res.v)
    print(f"track={TRACK} done_reason={res.done_reason} steps={n} "
          f"e_y_rms={np.mean(np.abs(np.array(res.e_y)) ** 2) ** 0.5:.4f} "
          f"progress_ratio={res.progress_ratio:.3f}")

    data = {
        "kind": "reference-core closed-loop demo",
        "controller": "linear_mpc (mpc_core reference, numpy ADMM)",
        "track": TRACK,
        "config_hash": MpcParams(**MPC_CFG).config_hash(),
        "ic": ic,
        "done_reason": res.done_reason,
        "completed": res.completed,
        "steps": n,
        "t_s": [round(k * TS, 4) for k in range(n + 1)],
        "ref": {"x": [float(v) for v in traj.x], "y": [float(v) for v in traj.y]},
        "run": {"x": [float(v) for v in xs], "y": [float(v) for v in ys],
                "e_y": res.e_y, "v": res.v, "omega": res.omega},
        "commit": git_head(),
    }
    jp = outdir / "circle_closed_loop.json"
    jp.write_text(json.dumps(data), encoding="utf-8")
    print("wrote", jp)

    if args.skip_render:
        return 0

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import PillowWriter

    rx = np.array(traj.x)
    ry = np.array(traj.y)
    ax_ = np.array(xs)
    ay_ = np.array(ys)
    ey = np.array(res.e_y)

    frames = args.frames
    step = max(1, n // frames)
    ks = list(range(0, n + 1, step))
    if ks[-1] != n:
        ks.append(n)

    fig, (a1, a2) = plt.subplots(
        1, 2, figsize=(9.4, 4.2), dpi=96,
        gridspec_kw={"width_ratios": [2.0, 1.0]})
    fig.subplots_adjust(left=0.055, right=0.985, top=0.90, bottom=0.13)

    # left: path + executed trajectory
    a1.plot(rx, ry, color="#9aa5b1", lw=1.6, ls="--", label="reference (circle R=2)")
    (line,) = a1.plot([], [], color="#2563eb", lw=2.2, label="MPC closed loop")
    (head,) = a1.plot([], [], "o", color="#dc2626", ms=6, zorder=5)
    (errline,) = a1.plot([], [], color="#f59e0b", lw=0.9, alpha=0.85)
    a1.set_aspect("equal")
    a1.set_title("Reference-core MPC closed loop - circle R=2 (offline demo)", fontsize=9)
    a1.legend(loc="upper right", fontsize=7.5, framealpha=0.9)
    a1.grid(alpha=0.25)

    # right: metrics panel as text
    a2.axis("off")
    txt = a2.text(0.02, 0.98, "", va="top", ha="left",
                  fontfamily="monospace", fontsize=8.6)

    fps = 12
    writer = PillowWriter(fps=fps)

    def near_ref(px, py):
        i = int(np.argmin((rx - px) ** 2 + (ry - py) ** 2))
        return rx[i], ry[i]

    def draw(k):
        kk = min(k, n)
        seg = ks[: ks.index(kk) + 1] if kk in ks else [q for q in ks if q <= kk] + [kk]
        # find insertion of kk
        idx = int(np.searchsorted(ks, kk))
        draw_ks = ks[: idx + 1]
        if draw_ks[-1] != kk:
            draw_ks.append(kk)
        sx = [ax_[q] for q in draw_ks]
        sy = [ay_[q] for q in draw_ks]
        line.set_data(sx, sy)
        head.set_data([ax_[kk]], [ay_[kk]])
        nrx, nry = near_ref(ax_[kk], ay_[kk])
        errline.set_data([ax_[kk], nrx], [ay_[kk], nry])
        t = kk * TS
        rmse_now = float(np.sqrt(np.mean(ey[:kk] ** 2))) if kk > 0 else 0.0
        emm = abs(ey[kk - 1]) * 1000.0 if kk > 0 else 0.0
        done = res.done_reason if kk >= n else ""
        txt.set_text(
            f"t = {t:5.1f} s     k = {kk:4d}\n"
            f"run e_y_rms = {rmse_now * 1000:6.1f} mm\n"
            f"e_y now     = {emm:6.1f} mm\n"
            f"driven ~    = {t * float(np.median(res.v[:kk + 1])) if kk else 0:5.1f} m\n"
            f"QP solve = numpy ADMM\n"
            f"plant    = perfect vel. track\n"
            f"{done}")
        return [line, head, errline, txt]

    def init():
        line.set_data([], [])
        head.set_data([], [])
        errline.set_data([], [])
        txt.set_text("")
        return [line, head, errline, txt]

    import matplotlib.animation as manim
    anim = manim.FuncAnimation(fig, draw, frames=ks, init_func=init,
                               interval=1000 // fps, blit=True)
    gp = outdir / "circle_closed_loop.gif"
    anim.save(gp, writer=writer)
    print("wrote", gp, f"({gp.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
