#!/usr/bin/env python3
"""Render the README "projection mismatch on a fold-back path" comparison.

Same pose sequence is replayed through the TWO real projection chains of the
reference core (LinearMpcController.controller_projection_mode):

  * "global"   -- stateless nearest-point projection; every raw arc accepted
                  as the new baseline (A8 instrumentation mode, pre-A5).
  * "windowed" -- stateful windowed projection (A2 window + heading gate) +
                  A5.1 acceptance gate (physical-displacement allowance) +
                  A4.2 reacquire protocol (NORMAL -> SEEKING -> PROBATION).

The pose sequence comes from ONE windowed closed-loop run on a fold-back
(u-turn) reference track, with two declared perturbations:

  1. a mid-gap lateral dip while driving the return strand (both strands of
     the fold are near the robot -> the stateless nearest point can snap to
     the opposite strand);
  2. a parked "kidnap" pose on the opposite (outbound) strand (a projection
     jump unreachable in one cycle).

Both chains then see the IDENTICAL pose sequence; everything plotted is a
recorded output of the real modules.

Honest labels:
  * this is a PROJECTION-CHAIN REPLAY COMPARISON over one declared pose
    sequence - NOT a closed-loop performance comparison of two controllers;
  * the pose sequence is synthetic / recorded offline (see scenario block),
    not a Gazebo run.

Outputs (into results/demo_20260909/):
  projection_foldback_replay.json  -- per-beat traces of both chains
  projection_foldback_replay.png   -- the comparison figure

Usage: python3 tools/make_projection_figure.py [--out results/demo_20260909]
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from benchmark_tools.scripts.run_reference_benchmark import MPC_CFG  # noqa: E402
from mpc_core.frenet import closest_point  # noqa: E402
from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from mpc_core.mpc import LinearMpcController  # noqa: E402
from mpc_core.types import KinematicState, MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import make_u_turn  # noqa: E402

TS = 0.05
V_REF = 0.5
JUMP_ARC_THRESH_M = 0.35   # accepted-arc beat change above this = a "jump"
COMPLETE_MARGIN_M = 0.15


def git_head() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True,
            text=True, check=True, cwd=ROOT).stdout.strip()
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------
# scenario: fold-back reference + declared pose-sequence perturbations
# --------------------------------------------------------------------------

def build_track():
    """Fold-back: straight outbound (y=0) -> 180 deg left arc -> straight
    return strand at y=1.0 driven in the -x direction.  The two strands are
    antiparallel and 1.0 m apart."""
    return make_u_turn(approach=2.5, radius=0.5, exit_l=2.5, v=V_REF, ds=0.02)


def run_nominal_windowed(traj, params, max_steps=3000):
    """Closed loop with the real windowed controller.  Records the pose fed
    to the controller at every beat.  Returns (states, summary)."""
    ctrl = LinearMpcController(MpcParams(**params), traj)
    plant = DifferentialDrivePlant(x0=-0.40, y0=0.0, yaw0=0.0,
                                   v0=0.0, omega0=0.0, seed=None)
    states = []
    accepted = 0.0
    path_len = float(traj.s[-1])
    stall = 0
    k = 0
    while k < max_steps:
        st = plant.state
        states.append((float(st.x), float(st.y), float(st.yaw),
                       float(st.v), float(st.omega)))
        out = ctrl.compute_cycle(st)
        plant.step(out.v_cmd, out.omega_cmd, TS)
        acc = float(out.diag.accepted_arc or 0.0)
        if acc != accepted:
            accepted = acc
            stall = 0
        else:
            stall += 1
        k += 1
        if accepted >= path_len - COMPLETE_MARGIN_M:
            break
        if out.diag.reason in ("PROJECTION_AMBIGUOUS", "PROJECTION_LOST",
                               "no reference set"):
            break
        if stall > 600:
            break
    return states, dict(steps=k, accepted=accepted, path_len=path_len)


def perturb(states):
    """Declared pose sequence S = nominal trace, but
    (1) the return-strand slice with x in (0.9, 1.7) is pulled down to the
        mid-gap y=0.45 (closer to the OUTBOUND strand than to the return
        strand);
    (2) a parked kidnap block on the outbound strand at (1.30, 0.02, yaw=0)
        for 28 beats, then a short forward drift on that strand.
    """
    x = np.array([s[0] for s in states])
    y = np.array([s[1] for s in states])
    yaw = np.array([s[2] for s in states])
    v = np.array([s[3] for s in states])
    w = np.array([s[4] for s in states])

    dip = (y > 0.75) & (x > 0.9) & (x < 1.7)
    y2 = y.copy()
    y2[dip] = 0.45
    s2 = list(zip(x.tolist(), y2.tolist(), yaw.tolist(),
                  v.tolist(), w.tolist()))

    jx, jy, jyaw = 1.30, 0.02, 0.0
    for _ in range(28):
        s2.append((jx, jy, jyaw, 0.0, 0.0))
    for _ in range(6):
        jx += 0.03
        s2.append((jx, jy, jyaw, 0.2, 0.0))

    dip_idx = [int(i) for i in np.where(dip)[0]]
    return s2, dict(nominal=len(states), dip_beats=dip_idx,
                    kidnap_start=len(states),
                    kidnap_pose=(1.30, 0.02, 0.0))


# --------------------------------------------------------------------------
# replay both real chains over the same pose sequence
# --------------------------------------------------------------------------

def replay_chain(mode, traj, poses, params):
    """Feed the identical pose sequence to a real LinearMpcController running
    projection mode `mode`; record every beat's public outputs."""
    p = dict(params)
    p["controller_projection_mode"] = mode
    ctrl = LinearMpcController(MpcParams(**p), traj)
    rows = []
    prev_arc = None
    for (x, y, yaw, v, w) in poses:
        st = KinematicState(x=x, y=y, yaw=yaw, v=v, omega=w)
        out = ctrl.compute_cycle(st)
        d = out.diag
        arc = float(d.accepted_arc or 0.0)
        seg, _w, ey_raw, arc_raw, stage = closest_point(traj, x, y)
        rows.append(dict(
            x=x, y=y, yaw=yaw, v=v, omega=w,
            arc=arc,
            d_arc=(arc - prev_arc) if prev_arc is not None else 0.0,
            reason=d.reason,
            health=str(getattr(d.health, "value", d.health)),
            reacquire_count=int(d.reacquire_count),
            in_probation=bool(getattr(d, "in_probation", False)),
            seg_raw=seg, arc_raw=arc_raw, ey_raw=ey_raw, stage_raw=stage,
            v_cmd=float(out.v_cmd),
            mode=_mode_of(ctrl),
            reject_run=int(getattr(ctrl, "_reject_run", 0)),
        ))
        prev_arc = arc
    return rows


def _mode_of(ctrl):
    m = getattr(ctrl, "_reacquire_mode", "NORMAL")
    return {"NORMAL": "NORMAL", "SEEKING": "SEEKING",
            "PROBATION": "PROBATION"}.get(m, str(m))


def e_y_anchor(traj, row):
    """Signed lateral distance of the pose from the path tangent at the
    ACCEPTED arc (same arithmetic the controller anchor uses)."""
    pt = traj.sample_by_s(row["arc"])
    dx = row["x"] - pt.x
    dy = row["y"] - pt.y
    return -dx * math.sin(pt.yaw) + dy * math.cos(pt.yaw)


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------

def render(traj, poses, g_rows, w_rows, meta, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    rx = np.asarray(traj.x)
    ry = np.asarray(traj.y)
    n = len(poses)

    g_arc = np.array([r["arc"] for r in g_rows])
    w_arc = np.array([r["arc"] for r in w_rows])
    g_ey = np.array([e_y_anchor(traj, r) for r in g_rows])
    w_ey = np.array([e_y_anchor(traj, r) for r in w_rows])
    w_v = np.array([r["v_cmd"] for r in w_rows])
    w_mode = np.array([r["mode"] for r in w_rows])
    g_jumps = [k for k in range(1, n)
               if abs(g_arc[k] - g_arc[k - 1]) > JUMP_ARC_THRESH_M]
    w_jumps = [k for k in range(1, n)
               if abs(w_arc[k] - w_arc[k - 1]) > JUMP_ARC_THRESH_M]
    seeking = [k for k in range(n) if w_mode[k] == "SEEKING"]
    prob = [k for k in range(n) if w_mode[k] == "PROBATION"]
    rejects = [k for k in range(1, n)
               if 1 <= w_rows[k]["reject_run"] <= 5 and w_mode[k] == "NORMAL"
               and abs(w_rows[k]["arc_raw"] - w_arc[k - 1]) > 0.2
               and abs(w_arc[k] - w_arc[k - 1]) < 1e-9]
    dip = meta["scenario"]["dip_beats"]
    ks = meta["scenario"]["kidnap_start"]
    tt = np.arange(n) * TS

    fig = plt.figure(figsize=(13.6, 8.6), dpi=110)
    gs = fig.add_gridspec(3, 2, height_ratios=[1.30, 0.95, 0.66],
                          hspace=0.46, wspace=0.14,
                          left=0.055, right=0.975, top=0.905, bottom=0.075)

    col_global = "#f59e0b"
    col_win = "#059669"
    col_pose = "#2563eb"
    col_dot = "#0d9488"

    # -- top row: XY panels ------------------------------------------------
    for ci, (rows, arc, jumps, title, sub, extra) in enumerate((
            (g_rows, g_arc, g_jumps,
             "Left: stateless nearest-point chain  (A8 'global' - raw arc accepted)",
             "per-beat projection/anchor feet; strand flip = jump booked as progress",
             "g"),
            (w_rows, w_arc, w_jumps,
             "Right: stateful projection + acceptance gate  (A2 + A5.1 + A4.2)",
             "accepted feet stay on the driven strand; unreachable jump rejected, "
             "then reacquired",
             "w"))):
        ax = fig.add_subplot(gs[0, ci])
        for k in range(len(rx) - 1):
            c = "#4b5563" if traj.s[k] < 2.5 else "#9ca3af"
            ax.plot(rx[k:k + 2], ry[k:k + 2], color=c, lw=1.8, zorder=1,
                    solid_capstyle="round")
        px = [r["x"] for r in rows]
        py = [r["y"] for r in rows]
        ax.plot(px, py, color=col_pose, lw=1.0, alpha=0.5, zorder=2,
                label="replayed pose sequence")
        axk = np.array([traj.sample_by_s(r["arc"]).x for r in rows])
        ayk = np.array([traj.sample_by_s(r["arc"]).y for r in rows])
        ax.plot(axk, ayk, ".", color=col_dot, ms=3.6, alpha=0.9, zorder=3,
                label="anchor feet (accepted arc)")
        ax.plot(px[0], py[0], "o", color="#111827", ms=5, zorder=5)
        ax.plot(px[-1], py[-1], "s", color="#111827", ms=5, zorder=5)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=9.3, pad=3)
        ax.set_xlabel(sub, fontsize=7.6)
        ax.grid(alpha=0.2)

        if extra == "g":
            # strand-flip jumps
            for k in jumps:
                ax.plot(rows[k]["x"], rows[k]["y"], "x", color="#dc2626",
                        ms=10, mew=1.9, zorder=6)
            if dip:
                k0 = dip[0]
                ax.annotate("mid-gap dip: nearest point snaps to the OUTBOUND "
                            "strand", xy=(rows[k0]["x"], rows[k0]["y"]),
                            xytext=(rows[k0]["x"] - 0.45, rows[k0]["y"] - 1.05),
                            fontsize=7.4, color="#b45309",
                            arrowprops=dict(arrowstyle="->", color="#b45309",
                                            lw=0.9))
            ax.text(0.02, 0.985, f"accepted-arc jumps: {len(jumps)}",
                    transform=ax.transAxes, fontsize=7.6, va="top",
                    color="#b91c1c", fontfamily="monospace")
        else:
            for k in rejects:
                ax.plot(rows[k]["x"], rows[k]["y"], "x", color="#dc2626",
                        ms=7.5, mew=1.4, zorder=6)
            if seeking:
                k0, k1 = seeking[0], seeking[-1]
                ax.axvspan(rows[k0]["x"] - 0.06, rows[k1]["x"] + 0.06,
                           color="#fecaca", alpha=0.4, zorder=0)
                ax.annotate("SEEKING: stop and re-search\n(rejects NORMAL)",
                            xy=(rows[k0]["x"], rows[k0]["y"]),
                            xytext=(rows[k0]["x"] + 0.35, rows[k0]["y"] + 0.30),
                            fontsize=7.4, color="#b91c1c",
                            arrowprops=dict(arrowstyle="->", color="#b91c1c",
                                            lw=0.9))
            if prob:
                k0, k1 = prob[0], prob[-1]
                ax.axvspan(rows[k0]["x"] - 0.06, rows[k1]["x"] + 0.06,
                           color="#bbf7d0", alpha=0.45, zorder=0)
                ax.annotate("PROBATION: capped speed\nobservation window",
                            xy=(rows[k1]["x"], rows[k1]["y"]),
                            xytext=(rows[k1]["x"] + 0.30, rows[k1]["y"] - 0.75),
                            fontsize=7.4, color="#166534",
                            arrowprops=dict(arrowstyle="->", color="#166534",
                                            lw=0.9))
            for k in range(1, len(rows)):
                if rows[k]["reacquire_count"] > rows[k - 1]["reacquire_count"]:
                    ax.plot(rows[k]["x"], rows[k]["y"], "*", color="#7c3aed",
                            ms=13, zorder=7)
                    ax.annotate("atomic re-anchor commit", xy=(rows[k]["x"], rows[k]["y"]),
                                xytext=(rows[k]["x"] - 0.30, rows[k]["y"] - 0.42),
                                fontsize=7.2, color="#6d28d9")
                    break
            ax.text(0.02, 0.985, f"accepted-arc jumps: {len(w_jumps)} "
                    "(1 = the re-anchor commit)",
                    transform=ax.transAxes, fontsize=7.6, va="top",
                    color="#065f46", fontfamily="monospace")
        ax.legend(loc="upper right", fontsize=6.8, framealpha=0.9)

    # -- bottom-left: accepted arc ledger ----------------------------------
    ax = fig.add_subplot(gs[1, :])
    ax.plot(tt, g_arc, color=col_global, lw=1.6,
            label="global: raw arc accepted as-is (stateless)")
    ax.plot(tt, w_arc, color=col_win, lw=1.6,
            label="windowed: accepted arc (gated)")
    ax.axhline(meta["path_len"], color="#9ca3af", lw=0.8, ls=":")
    for k in g_jumps:
        ax.plot([k * TS], [g_arc[k]], "x", color="#dc2626", ms=6, mew=1.4)
    if dip:
        ax.axvspan(tt[dip[0]], tt[dip[-1]], color="#fed7aa", alpha=0.35)
    if seeking:
        ax.axvspan(tt[seeking[0]], tt[seeking[-1]], color="#fecaca", alpha=0.5)
    if prob:
        ax.axvspan(tt[prob[0]], tt[prob[-1]], color="#bbf7d0", alpha=0.6)
    ax.set_ylabel("accepted arc / m", fontsize=8.6)
    ax.set_title("Progress ledger over the SAME pose sequence "
                 "(orange = strand flip booked; green = gated)", fontsize=9.3)
    ax.legend(fontsize=7.4, loc="lower right", ncol=2)
    ax.grid(alpha=0.25)
    ax.set_ylim(-0.4, float(meta["path_len"]) + 1.0)
    if dip:
        ax.text(tt[dip[0]] + 0.02, float(meta["path_len"]) + 0.55,
                "mid-gap dip (wrong-strand snap zone)", fontsize=7.2,
                color="#92400e")
    ax.text(tt[ks] - 0.15, float(meta["path_len"]) + 0.55,
            "kidnap on outbound strand", fontsize=7.2, color="#7f1d1d")

    # -- bottom-right: lateral error + speed command ------------------------
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(tt, g_ey, color=col_global, lw=1.2,
            label="global e_y vs its raw anchor")
    ax.plot(tt, w_ey, color=col_win, lw=1.2,
            label="windowed e_y vs its accepted anchor")
    ax.axhline(0.0, color="#9ca3af", lw=0.7)
    ax.set_ylabel("e_y / m", fontsize=8.6)
    ax.set_title("Lateral error vs the anchor each chain trusts", fontsize=9.0)
    ax.legend(fontsize=7.0)
    ax.grid(alpha=0.25)

    ax = fig.add_subplot(gs[2, 1])
    ax.plot(tt, w_v, color="#1d4ed8", lw=1.3, label="windowed v_cmd")
    for k in seeking:
        ax.plot([k * TS], [w_v[k]], ".", color="#dc2626", ms=6)
    if seeking:
        ax.axvspan(tt[seeking[0]], tt[seeking[-1]], color="#fecaca", alpha=0.5)
    if prob:
        ax.axvspan(tt[prob[0]], tt[prob[-1]], color="#bbf7d0", alpha=0.6)
    ax.set_ylabel("v_cmd / (m/s)", fontsize=8.6)
    ax.set_title("Right-chain speed command: reject -> 0, SEEKING -> 0, "
                 "PROBATION cap", fontsize=9.0)
    ax.legend(fontsize=7.0)
    ax.grid(alpha=0.25)

    fig.suptitle(
        "Projection-chain replay on a fold-back path - SAME pose sequence\n"
        "stateless nearest point  vs  stateful projection + acceptance gate "
        "(A5.1/A4.2)   |   module replay, NOT a closed-loop controller comparison",
        fontsize=10.5, y=0.985)
    png = outdir / "projection_foldback_replay.png"
    fig.savefig(png, dpi=110, facecolor="white")
    print("wrote", png, f"({png.stat().st_size / 1e3:.0f} KB)")
    return png


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "results" / "demo_20260909"))
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--skip-render", action="store_true")
    args = ap.parse_args()

    outdir = pathlib.Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    params = dict(MPC_CFG)
    traj = build_track()
    path_len = float(traj.s[-1])

    states, nom = run_nominal_windowed(traj, params, args.max_steps)
    print(f"nominal windowed run: steps={nom['steps']} "
          f"accepted={nom['accepted']:.3f} path_len={path_len:.3f}")
    poses, scenario = perturb(states)

    g_rows = replay_chain("global", traj, poses, params)
    w_rows = replay_chain("windowed", traj, poses, params)
    print(f"replay beats={len(poses)}")

    def jump_count(rows):
        return sum(1 for k in range(1, len(rows))
                   if abs(rows[k]["arc"] - rows[k - 1]["arc"])
                   > JUMP_ARC_THRESH_M)

    def regress_m(rows):
        return sum(1 for k in range(1, len(rows))
                   if rows[k]["arc"] - rows[k - 1]["arc"] < -0.02)

    print(f"accepted-arc jumps: global={jump_count(g_rows)} "
          f"windowed={jump_count(w_rows)}")
    print(f"backward-arc beats: global={regress_m(g_rows)} "
          f"windowed={regress_m(w_rows)}")

    meta = dict(
        kind="projection-chain replay comparison (fold-back path, same pose "
             "sequence)",
        chains=["global (stateless nearest, raw arc accepted)",
                "windowed (A2 window+heading gate / A5.1 gate / A4.2 "
                "reacquire)"],
        path_len=path_len,
        commit=git_head(),
        note="Same synthetic pose sequence replayed through both real "
             "reference-core chains; NOT a closed-loop performance "
             "comparison. Poses: one windowed closed-loop run on the "
             "fold-back track + declared perturbations (mid-gap dip on the "
             "return strand; parked kidnap pose on the outbound strand).",
        scenario=scenario,
        metrics=dict(
            global_jumps=jump_count(g_rows),
            windowed_jumps=jump_count(w_rows),
            global_backward_arc_beats=regress_m(g_rows),
            windowed_backward_arc_beats=regress_m(w_rows),
        ),
    )

    def compact(rows):
        return [{k: (round(v, 6) if isinstance(v, float) else v)
                 for k, v in r.items()} for r in rows]

    jp = outdir / "projection_foldback_replay.json"
    jp.write_text(json.dumps(dict(meta=meta, traj=dict(
        s=traj.s.tolist(), x=traj.x.tolist(), y=traj.y.tolist()),
        global_rows=compact(g_rows), windowed_rows=compact(w_rows)),
        ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", jp)

    if not args.skip_render:
        render(traj, poses, g_rows, w_rows, meta, outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
