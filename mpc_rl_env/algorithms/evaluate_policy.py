#!/usr/bin/env python3
"""Evaluate pure MPC vs MPC+PPO residual (U9/C7, AE4) with the no-safety
projection ablation (C8, AE6/AE7).  Requires torch + SB3 (RL venv).

Protocol (R21-R23): unseen seeds only, one env per (controller, track,
seed); controllers share identical tracks/initial conditions/MPC params;
every run lands in eval_runs/ as JSON and an aggregate MD/JSON is written.

Usage:
    python mpc_rl_env/algorithms/evaluate_policy.py \
        --checkpoint outputs/ppo_residual/checkpoint.zip \
        --tracks circle,s_curve --seeds 1,2,3,4
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import generate_benchmark_tracks  # noqa: E402


def build_env(track: str, cfg: dict, use_projection: bool):
    from mpc_core.types import MpcParams as MP
    from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
    from mpc_rl_env.envs.reward import RewardWeights

    traj = generate_benchmark_tracks()[track]
    return ResidualTrackingEnv(
        traj,
        mpc_params=MP(N=25),
        reward_w=RewardWeights(**cfg["reward_weights"]),
        alpha_residual=cfg["alpha_residual"],
        difficulty=cfg["env"]["difficulty"],
        use_projection=use_projection,
    )


def rollout(env, action_fn, max_steps: int, seed: int) -> dict:
    obs, _ = env.reset(seed=seed)  # identical initial conditions per seed (AE4)
    e_y, e_psi, du, triggers = [], [], [], 0
    completed, reason = False, "max_steps"
    prev_cmd = np.zeros(2)
    for i in range(max_steps):
        a = action_fn(obs)
        obs, _, terminated, info = env.step(a)
        e_y.append(abs(info["e_y"]))
        e_psi.append(abs(info["e_psi"]))
        du.append(float(np.sum((env._prev_cmd - prev_cmd) ** 2)))
        prev_cmd = env._prev_cmd
        triggers = info["projection_triggers"]
        if terminated:
            completed = info["reason"] == "completed"
            reason = info["reason"]
            break
    e_y = np.array(e_y)
    e_psi = np.array(e_psi)
    du = np.array(du)
    return {
        "steps": len(e_y),
        "completed": bool(completed),
        "termination_reason": reason,
        "e_y_rms": float(np.sqrt(np.mean(e_y**2))) if len(e_y) else None,
        "e_y_p95": float(np.percentile(e_y, 95)) if len(e_y) else None,
        "e_y_max": float(e_y.max()) if len(e_y) else None,
        "e_psi_rms": float(np.sqrt(np.mean(e_psi**2))) if len(e_psi) else None,
        "du_smooth": float(np.mean(du)) if len(du) else None,
        "projection_triggers": int(triggers),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default="outputs/ppo_residual/checkpoint.zip")
    ap.add_argument("--tracks", default="circle,s_curve")
    ap.add_argument("--seeds", default="1,2,3,4")
    ap.add_argument("--outdir", default="outputs/eval_residual")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    cfg_path = os.path.join(os.path.dirname(args.checkpoint), "run_config.yaml")
    if not os.path.isfile(cfg_path):
        sys.exit(f"missing {cfg_path} (R20 binding); retrain or copy run_config.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)["cfg"]

    model = PPO.load(args.checkpoint)
    tracks = [t for t in args.tracks.split(",") if t]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    max_steps = int(cfg["env"]["max_episode_steps"])

    def zero(_obs):
        return np.zeros(2)

    def policy(obs):
        a, _ = model.predict(obs, deterministic=True)
        return a

    controllers = {
        "mpc": (zero, True),
        "mpc_ppo": (policy, True),
        "mpc_ppo_no_safety": (policy, False),
    }

    results = []
    for cname, (afn, use_proj) in controllers.items():
        for track in tracks:
            for seed in seeds:
                env = build_env(track, cfg, use_proj)
                r = rollout(env, afn, max_steps, seed)
                r.update(controller=cname, track=track, seed=seed)
                results.append(r)
                print(f"[eval] {cname} {track} seed={seed}: "
                      f"{'OK ' if r['completed'] else 'X  '} "
                      f"e_y_rms={r['e_y_rms']:.3f} p95={r['e_y_p95']:.3f} "
                      f"reason={r['termination_reason']}")

    os.makedirs(args.outdir, exist_ok=True)
    with open(os.path.join(args.outdir, "eval_results.json"), "w") as f:
        json.dump({"checkpoint": args.checkpoint, "tracks": tracks,
                   "seeds": seeds, "results": results}, f, indent=1)

    lines = ["| controller | track | done | e_y_rms | e_y_p95 | smooth | proj_triggers |",
             "|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(
            f"| {r['controller']} | {r['track']} s{r['seed']} | "
            f"{'✅' if r['completed'] else '❌'} | {r['e_y_rms']:.3f} | "
            f"{r['e_y_p95']:.3f} | {r['du_smooth']:.4f} | {r['projection_triggers']} |")
    with open(os.path.join(args.outdir, "eval_results.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[eval] wrote {args.outdir}/eval_results.(json|md)")


if __name__ == "__main__":
    main()
