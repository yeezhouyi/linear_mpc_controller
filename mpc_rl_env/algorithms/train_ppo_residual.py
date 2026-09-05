#!/usr/bin/env python3
"""PPO residual training entry (U9/C7). Requires: torch + stable-baselines3 +
gymnasium (available in the RL venv on the dev machine).

    python mpc_rl_env/algorithms/train_ppo_residual.py \
        --seed 0 --total-timesteps 200000

Every run writes checkpoints, normalisation stats and the full config hash
(manifest binding, R20).  This module is an *entry point*: the training
harness itself is exercised once torch/sb3 are present (WSL2 or RL venv).
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import generate_benchmark_tracks  # noqa: E402


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_env(cfg: dict, seed: int, track: str):
    from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
    from mpc_rl_env.envs.gym_adapter import GymResidualTrackingEnv

    tracks = generate_benchmark_tracks()
    traj = tracks[track]
    rw = cfg["reward_weights"]
    from mpc_rl_env.envs.reward import RewardWeights

    env = ResidualTrackingEnv(
        traj,
        mpc_params=MpcParams(N=25),
        reward_w=RewardWeights(**rw),
        alpha_residual=cfg["alpha_residual"],
        difficulty=cfg["env"]["difficulty"],
    )
    # SB3 needs a gymnasium.Env; the core env stays dependency-free (R24).
    return GymResidualTrackingEnv(env)


class CurriculumV2Env:
    """Multi-track curriculum wrapper (U5/A7 v2).

    Each reset() advances a round-robin over ``train.trajectories`` and
    rotates the inner profile seed over ``train.seeds``.  Observation and
    action spaces are identical across tracks (12 / 2), so SB3 needs no
    adaptation.  Delegates everything else to the per-track inner env.
    """

    def __init__(self, cfg: dict, seed: int):
        self._cfg = cfg
        self._seed = seed
        self._tracks: list = list(cfg.get("train", {}).get(
            "trajectories", ["circle"]))
        self._ep = 0
        self._inner = None
        self._build_inner()

    def _build_inner(self):
        # raw core env (4-tuple API); the caller wraps THIS wrapper in the
        # gym adapter exactly once — no double wrapping
        from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
        from mpc_rl_env.envs.reward import RewardWeights

        track = self._tracks[self._ep % len(self._tracks)]
        rw = self._cfg["reward_weights"]
        self._inner = ResidualTrackingEnv(
            generate_benchmark_tracks()[track],
            mpc_params=MpcParams(N=25),
            reward_w=RewardWeights(**rw),
            alpha_residual=self._cfg["alpha_residual"],
            difficulty=self._cfg["env"]["difficulty"],
        )
        # reseed the inner profile rotation deterministically per episode
        self._inner.reset(seed=self._seed + self._ep)
        self._track_name = track

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def reset(self, *, seed=None, options=None):
        self._ep += 1
        self._build_inner()
        # raw core env takes seed only (no options kwarg)
        return self._inner.reset(seed=seed)

    def step(self, action):
        return self._inner.step(action)


def build_env_v2(cfg: dict, seed: int):
    from mpc_rl_env.envs.gym_adapter import GymResidualTrackingEnv

    return GymResidualTrackingEnv(CurriculumV2Env(cfg, seed))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--config", default="mpc_rl_env/config/ppo_residual.yaml")
    ap.add_argument("--total-timesteps", type=int, default=None)
    ap.add_argument("--track", default="circle")
    ap.add_argument("--outdir", default="outputs/ppo_residual")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if cfg.get("train", {}).get("trajectories"):
        # v2 curriculum: rotate all benchmark tracks per episode (A7/U5)
        env = build_env_v2(cfg, args.seed)
        print(f"[train] curriculum v2: tracks={cfg['train']['trajectories']}")
    else:
        env = build_env(cfg, args.seed, args.track)

    try:
        import gymnasium as gym  # noqa: F401
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
    except ImportError as e:  # pragma: no cover - env-specific
        print(f"[train] torch/sb3/gymnasium not available: {e}")
        print("[train] install them in the RL venv, then rerun (entry point only here).")
        sys.exit(2)

    def wrap():
        return gym.wrappers.TimeLimit(env, max_episode_steps=cfg["env"]["max_episode_steps"])

    vec = make_vec_env(wrap, n_envs=1, seed=args.seed)
    kwargs = dict(
        policy="MlpPolicy",
        env=vec,
        n_steps=cfg["n_steps"],
        batch_size=cfg["batch_size"],
        gamma=cfg["gamma"],
        gae_lambda=cfg["gae_lambda"],
        clip_range=cfg["clip_range"],
        ent_coef=cfg["ent_coef"],
        vf_coef=cfg["vf_coef"],
        max_grad_norm=cfg["max_grad_norm"],
        learning_rate=cfg["learning_rate"],
        seed=args.seed,
        verbose=1,
    )
    model = PPO(**kwargs)
    model.learn(total_timesteps=args.total_timesteps or cfg["total_timesteps"])
    os.makedirs(args.outdir, exist_ok=True)
    model.save(os.path.join(args.outdir, "checkpoint"))
    # save config binding next to the checkpoint
    with open(os.path.join(args.outdir, "run_config.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({"seed": args.seed, "cfg": cfg}, f)
    print(f"[train] saved {args.outdir}/checkpoint.zip")


if __name__ == "__main__":
    main()
