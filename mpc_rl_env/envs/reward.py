"""Reward function (R17, plan §RL MDP): error, progress, smoothness,
constraint/collision penalties -- never position error alone.

    r_t = -w_e (e_y^2 + lam_psi e_psi^2) - w_v e_v^2 - w_u ||du||^2
          + w_p ds - w_c * collision - w_q * constraint_violation - w_stall
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RewardWeights:
    w_e: float = 1.0
    lam_psi: float = 0.25
    w_v: float = 0.2
    w_u: float = 0.05
    w_p: float = 1.0
    w_c: float = 20.0
    w_q: float = 2.0
    w_stall: float = 1.0
    stall_threshold_v: float = 0.02


def compute_reward(
    w: RewardWeights,
    e_y: float,
    e_psi: float,
    e_v: float,
    du_norm2: float,
    ds: float,
    collision: bool,
    constraint_violation: float,
    stalled: bool,
    terminated: bool,
    completed: bool,
) -> float:
    r = -w.w_e * (e_y * e_y + w.lam_psi * e_psi * e_psi)
    r -= w.w_v * e_v * e_v
    r -= w.w_u * du_norm2
    r += w.w_p * ds
    if collision:
        r -= w.w_c
    r -= w.w_q * max(constraint_violation, 0.0)
    if stalled:
        r -= w.w_stall
    if completed:
        r += 5.0 * w.w_p  # sparse completion bonus
    return float(r)
