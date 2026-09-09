#!/usr/bin/env python3
"""Controller comparison: Frenet LTV-MPC vs Pure Pursuit (P0, review ruling).

SAME layer, SAME conditions: identical plant (differential drive, same lag),
identical references, identical limits (v_max / omega_max / a via clamps),
identical control period Ts=0.05, identical completion criterion and timeout
(accepted-arc watermark >= s_end - 0.15, A5.3), identical auditor (A5.2 gate).
Tuning is documented: PP lookahead Ld=0.30 m fixed (goal-bias 0.1 gain),
MPC uses the repo-standard MpcParams(N=25, Q/QF/S as shipped).

Scenarios (review table):
  straight_offset  straight reference, robot starts 0.30 m off-lane
  circle           R = 2 m circle (curvature tracking)
  s_curve          sinusoidal S bend (curvature tracking, sign flips)
  foldback         a8_cap serpentine (adjacent lanes: wrong-lane projection)
  foldback_lag     same + plant actuation lag 0.15 s (model mismatch)

QP failure rate / fallback counts are reported for the MPC ONLY (the ruling:
never compare a QP figure against controllers that have no QP).  Compute
latency p95/p99 is the controller's own per-cycle wall time.
"""
from __future__ import annotations

import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.episode import run_tracking_episode  # noqa: E402
from mpc_core.frenet import closest_point  # noqa: E402
from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from mpc_core.mpc import LinearMpcController  # noqa: E402
from mpc_core.types import (KinematicState, MpcDiagnostics, MpcOutput,  # noqa: E402
                            MpcParams, Trajectory, wrap_angle)


def make_traj(pts_x, pts_y, v_ref=0.3) -> Trajectory:
    x = np.asarray(pts_x, dtype=float)
    y = np.asarray(pts_y, dtype=float)
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
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=kappa,
                      v=np.full(n, v_ref), segment_gear=np.ones(max(1, n - 1)))


def build_scenarios():
    scen = {}
    # straight 6 m
    xs = np.arange(0, 6.0, 0.05)
    scen["straight_offset"] = (make_traj(xs, np.zeros_like(xs)),
                               dict(x0=0.0, y0=0.30, yaw0=0.0, lag_s=0.0))
    # circle R=2
    th = np.arange(0, 2 * math.pi + 0.025, 0.025)
    scen["circle"] = (make_traj(2 * np.sin(th), 2 - 2 * np.cos(th)),
                      dict(x0=0.0, y0=0.0, yaw0=0.0, lag_s=0.0))
    # S curve
    xs = np.arange(0, 8.0, 0.05)
    scen["s_curve"] = (make_traj(xs, 0.5 * np.sin(2 * math.pi * xs / 4.0)),
                       dict(x0=0.0, y0=0.0, yaw0=0.0, lag_s=0.0))
    # foldback serpentine: the a8_cap reference
    d = json.loads((Path("results/a8_replay/geo/a8_cap.json")).read_text())
    t = d["traj"]
    scen["foldback"] = (Trajectory(
        s=np.array(t["s"]), x=np.array(t["x"]), y=np.array(t["y"]),
        yaw=np.array(t["yaw"]), kappa=np.array(t["kappa"]),
        v=np.array(t["v"]), segment_gear=np.array(t["segment_gear"])),
        dict(x0=float(t["x"][0]), y0=float(t["y"][0]),
             yaw0=float(t["yaw"][0]), lag_s=0.0))
    # foldback with actuation lag 0.15 s
    scen["foldback_lag"] = (scen["foldback"][0],
                            dict(x0=float(t["x"][0]), y0=float(t["y"][0]),
                                 yaw0=float(t["yaw"][0]), lag_s=0.15))
    return scen


class PurePursuitController:
    """Drop-in pure-pursuit tracker (same limits, same projection source).

    Lookahead Ld = 0.30 m fixed (documented tuning); omega = 2 v sin(alpha)
    / Ld; v = reference speed at the lookahead arc, clamped to the SAME
    [v_min, v_max] bounds as the MPC.  Projection: the repo closest_point
    with s_prev continuity (no lane gate -- the foldback scenario shows
    what that costs, honestly)."""

    def __init__(self, params: MpcParams, Ld: float = 0.30):
        self.params = params
        self.Ld = Ld
        self.traj: Trajectory | None = None
        self.s_prev: float | None = None

    def set_reference(self, traj: Trajectory) -> None:
        self.traj = traj
        self.s_prev = None

    def compute_cycle(self, st: KinematicState) -> MpcOutput:
        t0 = time.perf_counter()
        _, _, _, arc, stage = closest_point(
            self.traj, st.x, st.y, yaw=st.yaw, s_prev=self.s_prev)
        self.s_prev = arc
        look_s = min(arc + self.Ld, self.traj.s[-1])
        tgt = self.traj.sample_by_s(look_s)
        v_ref = min(self.params.v_max,
                    max(abs(self.traj.sample_by_s(arc).v), 0.05))
        alpha = wrap_angle(math.atan2(tgt.y - st.y, tgt.x - st.x) - st.yaw)
        omega = 2.0 * v_ref * math.sin(alpha) / self.Ld
        omega = float(np.clip(omega, -self.params.omega_max,
                              self.params.omega_max))
        v = float(np.clip(v_ref, self.params.v_min, self.params.v_max))
        el = int((time.perf_counter() - t0) * 1e6)
        diag = MpcDiagnostics(
            health=__import__("mpc_core.types", fromlist=["HealthState"]).HealthState.OK,
            qp_status="N/A", qp_time_us=el)
        return MpcOutput(v_cmd=v, omega_cmd=omega, diag=diag)


def make_mpc(v_min: float, mode: str) -> LinearMpcController:
    p = MpcParams(N=25, qp_max_iter=500, v_min=v_min,
                  controller_projection_mode=mode)
    return LinearMpcController(p, None)


def summarize(res, ref_end: float, mpc: bool) -> dict:
    ey = np.abs(np.array(res.e_y))
    dv = np.abs(np.diff(np.array(res.cmd_v))) if len(res.cmd_v) > 1 else [0.0]
    dw = np.abs(np.diff(np.array(res.cmd_w))) if len(res.cmd_w) > 1 else [0.0]
    t = np.array(res.qp_time_us, dtype=float)
    return dict(
        done_reason=res.done_reason,
        progress_ratio=round(float(res.progress_ratio), 4),
        e_y_rms=round(float(np.sqrt(np.mean(np.array(res.e_y) ** 2))), 4),
        e_y_p95=round(float(np.percentile(ey, 95)), 4),
        end_gap_m=round(float(max(0.0, ref_end - res.arc_high_watermark)), 3),
        ctrl_variation=round(float(np.mean(dv) + np.mean(dw)), 4),
        latency_p95_us=int(np.percentile(t, 95)),
        latency_p99_us=int(np.percentile(t, 99)),
        qp_failures=(res.qp_failures if mpc else "N/A"),
        fallback_count=(res.fallback_count if mpc else "N/A"),
        steps=int(res.steps),
    )


def main() -> None:
    v_min = 0.0
    scen = build_scenarios()
    outdir = Path("results/controller_compare")
    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, (traj, init) in scen.items():
        for ctrl_name in ("mpc", "pure_pursuit"):
            plant = DifferentialDrivePlant(
                x0=init["x0"], y0=init["y0"], yaw0=init["yaw0"],
                lag_s=init["lag_s"])
            if ctrl_name == "mpc":
                controller = make_mpc(v_min, "windowed")
            else:
                controller = PurePursuitController(
                    MpcParams(N=25, v_min=v_min))
            controller.set_reference(traj)
            res = run_tracking_episode(controller, plant, traj,
                                       track_name=ctrl_name,
                                       audit_mode="accepted")
            s = summarize(res, float(traj.s[-1]), mpc=(ctrl_name == "mpc"))
            s.update(controller=ctrl_name, scenario=name)
            rows.append(s)
            print(json.dumps(s), flush=True)
            (outdir / (name + "_" + ctrl_name + ".json")).write_text(
                json.dumps(s, indent=1))
    # ---- aggregate ----
    agg = {}
    for name in scen:
        agg[name] = {}
        for ctrl in ("mpc", "pure_pursuit"):
            r = next(x for x in rows
                     if x["scenario"] == name and x["controller"] == ctrl)
            agg[name][ctrl] = r
    md = ["# Controller comparison: Frenet LTV-MPC vs Pure Pursuit", "",
          "Same plant / references / limits / Ts / completion criterion /",
          "auditor (A5.2 gate).  Ld = 0.30 m; MPC MpcParams as shipped.", "",
          "| scenario | controller | done | progress | e_y_rms | e_y_p95"
          " | end_gap_m | ctrl_var | lat_p95_us | lat_p99_us | qp_fail |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name in scen:
        for ctrl in ("mpc", "pure_pursuit"):
            r = agg[name][ctrl]
            md.append("| %s | %s | %s | %.4f | %.4f | %.4f | %.3f | %.4f"
                      " | %d | %d | %s |"
                      % (name, ctrl, r["done_reason"], r["progress_ratio"],
                         r["e_y_rms"], r["e_y_p95"], r["end_gap_m"],
                         r["ctrl_variation"], r["latency_p95_us"],
                         r["latency_p99_us"], str(r["qp_failures"])))
    (outdir / "controller_compare.md").write_text("\n".join(md) + "\n")
    (outdir / "controller_compare.json").write_text(
        json.dumps(agg, indent=1))
    print("written:", outdir / "controller_compare.md")


if __name__ == "__main__":
    main()
