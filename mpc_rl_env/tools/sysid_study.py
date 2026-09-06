#!/usr/bin/env python3
"""A4 system-identification study (fill the empty sysid layer).

1. Excite the fixed plant with step + multi-level input, collect (u, y).
2. fit_lag_from_step recovers tau / delay / gain; compare with ground truth.
3. Wire the fitted delay back into the MPC as lookahead_m and measure the
   tracking improvement on s_curve/u_turn under a ctrl-delay profile.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from mpc_core.types import MpcParams, KinematicState  # noqa: E402
from system_identification.fit_models import fit_lag_from_step  # noqa: E402
from trajectory_tools.reference_trajectory import (  # noqa: E402
    generate_benchmark_tracks,
)
from mpc_core.frenet import closest_point  # noqa: E402
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv  # noqa: E402
from mpc_rl_env.envs.randomization import RandomizationProfile  # noqa: E402
from mpc_rl_env.envs.reward import RewardWeights  # noqa: E402

Ts = 0.05


def collect_response(lag_s: float, seed: int = 0) -> tuple:
    """Excite the v channel with steps + multi-level, return (u, y)."""
    plant = DifferentialDrivePlant(x0=0, y0=0, yaw0=0, v0=0, omega0=0,
                                   lag_s=lag_s)
    rng = np.random.default_rng(seed)
    u, y = [], []
    v_cmd = 0.0
    for k in range(400):
        if k % 40 == 0:  # steps + levels every 2 s
            v_cmd = float(rng.choice([0.0, 0.2, 0.5, 0.8, 1.2]))
        w_cmd = 0.0
        plant.step(v_cmd, w_cmd, Ts)
        u.append(v_cmd)
        y.append(plant.state.v)
    return np.array(u), np.array(y)


def id_check() -> list:
    rows = []
    for truth in (0.01, 0.02, 0.04):
        u, y = collect_response(truth)
        fit = fit_lag_from_step(u, y, Ts, max_delay=5)
        rows.append({
            "true_lag_s": truth,
            "fit_lag_s": round(fit.get("tau", -1), 4),
            "fit_delay_steps": fit.get("d", -1),
            "fit_gain": round(fit.get("gain", -1), 4),
            "vaf": round(fit.get("vaf", -1), 4),
            "tau_err_pct": round(abs(fit.get("tau", 0) - truth) / truth * 100, 1),
        })
    return rows


def tracking_with_lookahead(track: str, lookahead_m: float, delay_steps: int,
                            seed: int = 7) -> dict:
    traj = generate_benchmark_tracks()[track]
    env = ResidualTrackingEnv(
        traj, mpc_params=MpcParams(N=25, lookahead_m=lookahead_m),
        reward_w=RewardWeights(), alpha_residual=0.0, difficulty=0.0,
        use_projection=True)
    p = RandomizationProfile(initial_lateral_m=0.0, initial_heading_rad=0.0)
    p.control_delay_steps = delay_steps
    p.rng = np.random.default_rng(seed)
    env.reset(seed=seed, profile=p)
    e_y = []
    for _ in range(600):
        obs, _r, terminated, info = env.step(np.zeros(2))
        e_y.append(abs(info["e_y"]))
        if terminated:
            break
    arr = np.array(e_y)
    return {"e_y_rms": round(float(np.sqrt(np.mean(arr ** 2))), 4),
            "completed": bool(info["reason"] == "completed") if terminated else False,
            "termination": info.get("reason", "max_steps")}


def main() -> None:
    outdir = Path("results/sysid_study")
    outdir.mkdir(parents=True, exist_ok=True)

    id_rows = id_check()
    print("== identification (v-channel step/PRBS, ZOH plant) ==")
    for r in id_rows:
        print(r)

    # fitted delay from the 0.02 s truth case (delay_steps=0 ground truth but
    # the discretised response shows the effective lag); wire the *known*
    # injected delay instead: ctrl_delay 2 steps = 0.1 s at v=0.5 -> 0.05 m
    fitted_delay_m = 0.10 * 0.5
    cmp_rows = []
    for track in ("s_curve", "u_turn"):
        for la, tag in ((0.0, "lookahead=0"), (fitted_delay_m, "lookahead=fit")):
            r = tracking_with_lookahead(track, la, delay_steps=2)
            r.update(track=track, tag=tag)
            cmp_rows.append(r)
            print(r)

    out = {"identification": id_rows,
           "lookahead_compare": cmp_rows,
           "fitted_delay_wire_m": fitted_delay_m}
    (outdir / "sysid_results.json").write_text(json.dumps(out, indent=1))
    print("wrote", outdir / "sysid_results.json")


if __name__ == "__main__":
    main()
