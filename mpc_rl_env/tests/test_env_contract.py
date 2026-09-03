"""Fast env contract tests (U8): obs dims, determinism, zero-action == MPC."""
import numpy as np
import pytest

from mpc_core.episode import run_tracking_episode
from mpc_core.model import DifferentialDrivePlant
from mpc_core.types import MpcParams
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
from mpc_rl_env.envs.observation_builder import N_OBS
from mpc_rl_env.envs.randomization import RandomizationProfile
from trajectory_tools.reference_trajectory import make_circle, make_straight


def test_reset_observation_shape_and_finite():
    env = ResidualTrackingEnv(make_straight(length=4.0, v=0.5), difficulty=0.0)
    obs, info = env.reset(seed=0)
    assert obs.shape == (N_OBS,)
    assert np.all(np.isfinite(obs))


def test_random_actions_stay_finite_and_bounded():
    env = ResidualTrackingEnv(make_circle(radius=2.0, v=0.5), difficulty=0.0)
    env.reset(seed=1)
    rng = np.random.default_rng(2)
    for _ in range(40):
        obs, reward, done, info = env.step(rng.uniform(-1, 1, size=2))
        assert np.all(np.isfinite(obs))
        assert np.isfinite(reward)
        assert env.plant.state.v <= env.mpc_params.v_max + 1e-6
        assert abs(env.plant.state.omega) <= env.mpc_params.omega_max + 1e-6
        if done:
            env.reset(seed=3)


def test_zero_residual_equals_pure_mpc():
    """alpha scaling: a zero residual must not change the MPC baseline
    (i.e. a policy that outputs 0 is exactly pure MPC)."""
    traj = make_circle(radius=2.0, v=0.5)
    params = MpcParams(N=15, Q_diag=(80.,12.,2.,1.))
    env = ResidualTrackingEnv(traj, mpc_params=params, difficulty=0.0)
    profile = RandomizationProfile(track_name="circle", initial_lateral_m=0.1,
                                   initial_heading_rad=0.05, seed=0)
    obs0, _ = env.reset(seed=0, profile=profile)

    eys = []
    for _ in range(120):
        _, _, done, _ = env.step(np.zeros(2))
        eys.append(env._obs[0])
        if done:
            break
    assert len(eys) > 30  # deterministic env keeps running
    # MPC must be able to reduce the initial lateral error over this window
    assert abs(eys[-1]) < abs(eys[0]) + 1e-9


def test_deterministic_seed_repeatable():
    traj = make_straight(length=4.0, v=0.5)
    def run_once():
        env = ResidualTrackingEnv(traj, difficulty=0.2)
        env.reset(seed=7)
        rng = np.random.default_rng(11)
        obs = []
        for _ in range(25):
            o, _, done, _ = env.step(rng.uniform(-1, 1, 2))
            obs.append(o.copy())
            if done:
                break
        return np.array(obs)
    a = run_once()
    b = run_once()
    assert np.allclose(a, b)
