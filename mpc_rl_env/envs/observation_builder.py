"""Observation builder (R16): only runtime-available quantities, normalised."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

N_OBS = 12


@dataclass
class ObsStats:
    """Normalisation statistics (fitted offline; stored with the checkpoint)."""

    mean: np.ndarray
    std: np.ndarray


def build_observation(
    err: np.ndarray,          # [e_y, e_psi, v, omega]
    ref_v: float,
    ref_kappa: float,
    prev_cmd: np.ndarray,     # last commanded (v, omega)
    mpc_health_ok: bool,
    obstacle_cost: float,     # 0 = free (runtime local cost if wired)
) -> np.ndarray:
    """Raw (unnormalised) observation vector:
    [e_y, e_psi, v, omega, ref_v, ref_kappa,
     e_y_dot, e_psi_dot, prev_cmd_v, prev_cmd_omega, health_ok, obstacle_cost]
    e_y_dot / e_psi_dot are finite differences supplied by the env.
    """
    obs = np.zeros(N_OBS)
    obs[0:4] = err
    obs[4] = ref_v
    obs[5] = ref_kappa
    # slots 6,7 filled by the env (finite differences)
    obs[8] = prev_cmd[0]
    obs[9] = prev_cmd[1]
    obs[10] = 1.0 if mpc_health_ok else 0.0
    obs[11] = float(obstacle_cost)
    return obs


def default_stats() -> ObsStats:
    """Hand-tuned normalisation (documented; replace after data collection)."""
    mean = np.zeros(N_OBS)
    std = np.ones(N_OBS)
    std[0] = 1.0    # e_y [m]
    std[1] = 0.5    # e_psi [rad]
    std[2] = 1.0    # v [m/s]
    std[3] = 1.0    # omega [rad/s]
    std[4] = 1.0    # ref_v
    std[5] = 1.0    # kappa (up to ~1/radius)
    std[6] = 1.0    # e_y_dot
    std[7] = 1.0    # e_psi_dot
    std[8] = 1.0    # prev v
    std[9] = 2.0    # prev omega
    std[10] = 1.0
    std[11] = 1.0
    return ObsStats(mean=mean, std=std)


def normalize(obs: np.ndarray, stats: ObsStats) -> np.ndarray:
    std = np.where(stats.std <= 1e-9, 1.0, stats.std)
    return (obs - stats.mean) / std
