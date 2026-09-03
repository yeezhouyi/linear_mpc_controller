"""Gymnasium adapter contract tests (U9).

Skipped when gymnasium is absent so the pure-python gate (system python,
no torch) stays green; exercised in the RL venv where training runs.
"""
import numpy as np
import pytest

gd = pytest.importorskip("gymnasium")

from mpc_core.types import MpcParams  # noqa: E402
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv  # noqa: E402
from mpc_rl_env.envs.gym_adapter import GymResidualTrackingEnv  # noqa: E402
from trajectory_tools.reference_trajectory import generate_benchmark_tracks  # noqa: E402


def _make_env() -> GymResidualTrackingEnv:
    traj = generate_benchmark_tracks()["circle"]
    return GymResidualTrackingEnv(ResidualTrackingEnv(traj, mpc_params=MpcParams(N=25)))


def test_spaces():
    env = _make_env()
    assert env.observation_space.shape == (12,)
    assert env.action_space.shape == (2,)
    assert np.all(env.action_space.low == -1.0)
    assert np.all(env.action_space.high == 1.0)


def test_reset_returns_obs_info():
    env = _make_env()
    obs, info = env.reset(seed=0)
    assert obs.shape == (12,)
    assert env.observation_space.contains(obs)
    assert isinstance(info, dict)


def test_step_returns_gymnasium_5tuple():
    env = _make_env()
    env.reset(seed=0)
    out = env.step(np.array([0.0, 0.0]))
    assert len(out) == 5
    obs, reward, terminated, truncated, info = out
    assert obs.shape == (12,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert truncated is False  # truncation owned by TimeLimit wrapper
    assert "reason" in info


def test_sb3_env_checker():
    env = _make_env()
    env.reset(seed=0)
    from stable_baselines3.common.env_checker import check_env

    check_env(env, warn=True)
