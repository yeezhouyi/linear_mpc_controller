"""Gymnasium adapter for ResidualTrackingEnv (U9/C7).

The core fast env is dependency-free by design (R24: one env contract for
pure-python gates and RL training).  When torch/sb3/gymnasium are present
(RL venv), this thin shim exposes it as a ``gymnasium.Env``:

* observation_space: Box(shape=(12,))  -- see observation_builder
* action_space:       Box([-1, 1]^2)   -- normalised residual delta_u
* step:               adds the gymnasium 5-tuple ``truncated`` slot
                       (truncation is owned by the TimeLimit wrapper)
"""
from __future__ import annotations

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces

    GYM_AVAILABLE = True
except ImportError:  # pragma: no cover - pure-python gate runs without gymnasium
    GYM_AVAILABLE = False

OBS_DIM = 12
ACT_DIM = 2


if GYM_AVAILABLE:

    class GymResidualTrackingEnv(gym.Env):
        """Passthrough shim: ResidualTrackingEnv -> gymnasium.Env."""

        metadata = {"render_modes": []}

        def __init__(self, env) -> None:
            self.env = env
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float64
            )
            self.action_space = spaces.Box(
                low=-1.0, high=1.0, shape=(ACT_DIM,), dtype=np.float64
            )

        def reset(self, *, seed: int | None = None, options: dict | None = None):
            obs, info = self.env.reset(seed=seed)
            return obs, info

        def step(self, action):
            obs, reward, terminated, info = self.env.step(action)
            return obs, reward, terminated, False, info

        def render(self):  # pragma: no cover - headless by design
            return None
