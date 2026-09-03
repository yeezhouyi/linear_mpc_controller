"""Domain randomisation (KTD8): interpretable parameters only.

Every run records the sampled profile so results can be stratified
(R20: config hash + seed + profile in the manifest).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

import numpy as np


@dataclass
class RandomizationProfile:
    track_name: str = ""
    wheel_radius_scale: float = 1.0
    wheel_base_scale: float = 1.0
    velocity_lag_s: float = 0.0      # first-order lag on cmd->actual
    control_delay_steps: int = 0     # integer delay of cmd application
    measure_noise_m: float = 0.0     # odom noise sigma on pose
    initial_lateral_m: float = 0.3
    initial_heading_rad: float = 0.15
    seed: int = 0
    rng: np.random.Generator = field(repr=False, default_factory=lambda: np.random.default_rng(0))

    def to_dict(self) -> Dict:
        d = self.__dict__.copy()
        d.pop("rng", None)
        return d


def sample_profile(rng: np.random.Generator, track_name: str, difficulty: float = 1.0) -> RandomizationProfile:
    """Sample one profile.  ``difficulty`` in [0..1] scales the ranges
    (curriculum learning hook, C6)."""
    p = RandomizationProfile(track_name=track_name, seed=int(rng.integers(0, 2**31 - 1)))
    p.rng = rng
    p.wheel_radius_scale = float(rng.uniform(1 - 0.08 * difficulty, 1 + 0.08 * difficulty))
    p.wheel_base_scale = float(rng.uniform(1 - 0.08 * difficulty, 1 + 0.08 * difficulty))
    p.velocity_lag_s = float(rng.uniform(0.0, 0.15 * difficulty))
    p.control_delay_steps = int(rng.integers(0, 1 + int(3 * difficulty)))
    p.measure_noise_m = float(rng.uniform(0.0, 0.02 * difficulty))
    p.initial_lateral_m = float(rng.uniform(0.0, 0.6 * difficulty))
    p.initial_heading_rad = float(rng.uniform(0.0, 0.35 * difficulty))
    return p
