#!/usr/bin/env python3
"""A3 single-variable attribution study (plan: explain offline-vs-RL-env gap).

Runs pure MPC in the fast env under controlled conditions:
  baseline      : zero randomization (difficulty 0 equivalent)
  init_error    : initial lateral 0.15 m only
  vel_lag       : velocity lag 0.04 s only
  ctrl_delay    : control delay 2 steps (0.1 s) only
  meas_noise    : measurement noise 0.005 m only
  full_random   : difficulty 0.3 sampled profile (the training condition)

4 tracks x 4 seeds per condition.  Emits per-cell completion/e_y_rms/
projection triggers and a markdown attribution table.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.types import MpcParams  # noqa: E402
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv  # noqa: E402
from mpc_rl_env.envs.randomization import RandomizationProfile  # noqa: E402
from mpc_rl_env.envs.reward import RewardWeights  # noqa: E402
from trajectory_tools.reference_trajectory import (  # noqa: E402
    generate_benchmark_tracks,
)

CONDITIONS = ("baseline", "init_error", "vel_lag", "ctrl_delay",
              "meas_noise", "full_random")
TRACKS = ("straight", "circle", "s_curve", "u_turn")
SEEDS = (0, 1, 2, 3)
STEPS = 600


def make_profile(condition: str, rng: np.random.Generator) -> RandomizationProfile:
    # NOTE: the dataclass defaults are NOT zero (initial_lateral_m=0.3,
    # initial_heading_rad=0.15) — the baseline must zero them explicitly.
    p = RandomizationProfile(
        initial_lateral_m=0.0, initial_heading_rad=0.0)
    p.rng = rng
    p.seed = int(rng.integers(0, 2 ** 31 - 1))
    if condition == "init_error":
        p.initial_lateral_m = 0.15
        p.initial_heading_rad = 0.05
    elif condition == "vel_lag":
        p.velocity_lag_s = 0.04
    elif condition == "ctrl_delay":
        p.control_delay_steps = 2
    elif condition == "meas_noise":
        p.measure_noise_m = 0.005
    elif condition == "full_random":
        s = sample_full(rng)
        return s
    return p


def sample_full(rng: np.random.Generator) -> RandomizationProfile:
    """difficulty 0.3 profile, same ranges as sample_profile."""
    p = RandomizationProfile()
    p.rng = rng
    p.seed = int(rng.integers(0, 2 ** 31 - 1))
    p.initial_lateral_m = float(rng.uniform(-0.05 * 0.3 - 0.15 * 0.3,
                                            0.15 * 0.3 + 0.05 * 0.3))
    p.initial_heading_rad = float(rng.uniform(-0.15 * 0.3, 0.15 * 0.3))
    p.velocity_lag_s = float(rng.uniform(0.0, 0.15 * 0.3))
    p.control_delay_steps = int(rng.integers(0, 3))
    p.wheel_radius_scale = float(rng.uniform(1 - 0.08 * 0.3, 1 + 0.08 * 0.3))
    p.wheel_base_scale = float(rng.uniform(1 - 0.08 * 0.3, 1 + 0.08 * 0.3))
    p.measure_noise_m = float(rng.uniform(0.0, 0.02 * 0.3))
    return p


def run_one(track: str, seed: int, condition: str) -> dict:
    traj = generate_benchmark_tracks()[track]
    env = ResidualTrackingEnv(
        traj,
        mpc_params=MpcParams(N=25),
        reward_w=RewardWeights(),
        alpha_residual=0.0,       # pure MPC: no residual fused
        difficulty=0.0,
        use_projection=True,
    )
    rng = np.random.default_rng(seed)
    profile = make_profile(condition, rng)
    obs, _ = env.reset(seed=seed, profile=profile)
    e_y, done, reason = [], False, "max_steps"
    for _ in range(STEPS):
        obs, _r, terminated, info = env.step(np.zeros(2))
        e_y.append(abs(info["e_y"]))
        if terminated:
            done = info["reason"] == "completed"
            reason = info["reason"]
            break
    e_y_arr = np.array(e_y) if e_y else np.array([np.nan])
    return {
        "track": track, "seed": seed, "condition": condition,
        "completed": bool(done), "termination": reason,
        "e_y_rms": float(np.sqrt(np.nanmean(e_y_arr ** 2))),
        "e_y_p95": float(np.nanpercentile(e_y_arr, 95)),
        "projection_triggers": int(env.projection_trigger_count),
    }


def main() -> None:
    outdir = Path("results/attribution_study")
    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    for condition in CONDITIONS:
        for track in TRACKS:
            for seed in SEEDS:
                r = run_one(track, seed, condition)
                rows.append(r)
                print("[%s] %s s%d: %s rms=%.3f p95=%.3f proj=%d" % (
                    condition, track, seed,
                    "OK " if r["completed"] else "X  ",
                    r["e_y_rms"], r["e_y_p95"], r["projection_triggers"]))

    (outdir / "runs.json").write_text(json.dumps(rows, indent=1))

    lines = ["| condition | track | completed | e_y_rms mean | e_y_p95 mean | proj mean |",
             "|---|---|---|---|---|---|"]
    for condition in CONDITIONS:
        for track in TRACKS:
            sel = [r for r in rows
                   if r["condition"] == condition and r["track"] == track]
            n_ok = sum(1 for r in sel if r["completed"])
            lines.append(
                "| %s | %s | %d/4 | %.3f | %.3f | %.0f |" % (
                    condition, track, n_ok,
                    float(np.mean([r["e_y_rms"] for r in sel])),
                    float(np.mean([r["e_y_p95"] for r in sel])),
                    float(np.mean([r["projection_triggers"] for r in sel]))))
    table = "\n".join(lines)
    (outdir / "attribution_table.md").write_text(table + "\n")
    print(table)


if __name__ == "__main__":
    main()
