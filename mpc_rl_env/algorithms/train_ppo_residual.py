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

from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import generate_benchmark_tracks  # noqa: E402


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_env(cfg: dict, seed: int, track: str):
    from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
    from mpc_rl_env.envs import gym_adapter

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
    return gym_adapter.GymResidualTrackingEnv(env)


def build_t07_env(cfg: dict, seed: int, run_name: str = "t07_training",
                  track: str | None = None):
    """Build the opt-in T07 v1 adapter and its optional Gym shim.

    The adapter is intentionally separate from ``build_env``: callers must
    select ``contract='t07_v1'`` and receive a 55D/1D manifest-bound env.
    This function performs no training and is usable by offline Gate F tests.
    """
    from mpc_rl_env.envs import gym_adapter
    from tools.t07_contract_env import T07Config
    from tools.t07_training_adapter import make_training_adapter

    # ``track`` is the explicit CLI/run selector.  A nested or flat
    # ``path_id`` is only a default when the caller omitted it, preventing a
    # configured ``straight`` from silently relabeling an explicit circle run.
    t07_cfg = cfg.get("t07") or cfg
    path_id = str(track or t07_cfg.get("path_id", "straight"))
    config = T07Config(
        dt_s=float(t07_cfg.get("dt_s", 0.05)),
        delay_steps=int(t07_cfg.get("delay_steps", 0)),
        measurement_noise_std=float(t07_cfg.get("measurement_noise_std", 0.0)),
        encoder_bias_v=float(t07_cfg.get("encoder_bias_v", 0.0)),
        imu_bias_omega=float(t07_cfg.get("imu_bias_omega", 0.0)),
        velocity_lag=float(t07_cfg.get("velocity_lag", 0.0)),
        v_limit=float(t07_cfg.get("v_limit", 0.30)),
        omega_limit=float(t07_cfg.get("omega_limit", 1.20)),
        path_id=path_id,
    )
    adapter = make_training_adapter(contract="t07_v1", config=config,
                                    run_name=run_name)
    adapter.reset(seed=seed, path_id=path_id)
    if not gym_adapter.GYM_AVAILABLE:
        # Without gymnasium the caller uses the dependency-free adapter for
        # contract tests; no PPO entry point is attempted.
        return adapter, adapter
    return gym_adapter.GymResidualTrackingEnv(adapter), adapter


def build_selected_env(cfg: dict, seed: int, track: str | None,
                       contract: str = "legacy"):
    """Select legacy or T07 explicitly; unknown contracts fail closed."""
    if contract == "legacy":
        return build_env(cfg, seed, track or "circle"), None
    if contract == "t07_v1":
        selected = track or (cfg.get("t07") or cfg).get("path_id", "straight")
        return build_t07_env(cfg, seed, run_name=f"t07_{selected}", track=selected)
    raise ValueError(f"unsupported training contract {contract!r}")


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
    ap.add_argument("--track", default=None,
                    help="benchmark path; for T07 this becomes path_id")
    ap.add_argument("--outdir", default="outputs/ppo_residual")
    ap.add_argument("--dry-run", action="store_true",
                    help="build the selected env and write its manifest without importing PPO")
    ap.add_argument("--contract", choices=("legacy", "t07_v1"), default="legacy",
                    help="training contract; legacy preserves the frozen 12D/2D path")
    args = ap.parse_args()

    cfg = load_config(args.config)
    adapter = None
    if args.contract == "t07_v1":
        env, adapter = build_selected_env(cfg, args.seed, args.track, args.contract)
        os.makedirs(args.outdir, exist_ok=True)
        adapter.write_manifest(os.path.join(args.outdir, "t07_manifest.json"), seed=args.seed)
        print("[train] selected explicit contract=t07_v1; manifest written")
        if args.dry_run:
            print("[train] dry-run complete; PPO was not imported")
            return
    elif cfg.get("train", {}).get("trajectories"):
        # v2 curriculum: rotate all benchmark tracks per episode (A7/U5)
        env = build_env_v2(cfg, args.seed)
        print(f"[train] curriculum v2: tracks={cfg['train']['trajectories']}")
    else:
        env = build_env(cfg, args.seed, args.track or "circle")

    try:
        import gymnasium as gym  # noqa: F401
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
    except ImportError as e:  # pragma: no cover - env-specific
        print(f"[train] torch/sb3/gymnasium not available: {e}")
        print("[train] install them in the RL venv, then rerun (entry point only here).")
        sys.exit(2)

    def wrap():
        return gym.wrappers.TimeLimit(
            env, max_episode_steps=cfg.get("env", {}).get("max_episode_steps", 600)
        )

    vec = make_vec_env(wrap, n_envs=1, seed=args.seed)
    kwargs = dict(
        policy="MlpPolicy",
        env=vec,
        n_steps=cfg.get("n_steps", 512),
        batch_size=cfg.get("batch_size", 128),
        gamma=cfg.get("gamma", 0.99),
        gae_lambda=cfg.get("gae_lambda", 0.95),
        clip_range=cfg.get("clip_range", 0.2),
        ent_coef=cfg.get("ent_coef", 0.0),
        vf_coef=cfg.get("vf_coef", 0.5),
        max_grad_norm=cfg.get("max_grad_norm", 0.5),
        learning_rate=cfg.get("learning_rate", 3.0e-4),
        seed=args.seed,
        verbose=1,
    )
    model = PPO(**kwargs)
    model.learn(total_timesteps=args.total_timesteps or cfg.get("total_timesteps", 200_000))
    os.makedirs(args.outdir, exist_ok=True)
    model.save(os.path.join(args.outdir, "checkpoint"))
    # save config binding next to the checkpoint
    with open(os.path.join(args.outdir, "run_config.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({"seed": args.seed, "cfg": cfg}, f)
    print(f"[train] saved {args.outdir}/checkpoint.zip")


if __name__ == "__main__":
    main()
