"""Termination rules (R14)."""
from __future__ import annotations

from dataclasses import dataclass

from mpc_core.types import HealthState


@dataclass
class TerminationConfig:
    max_steps: int = 1200
    complete_margin_m: float = 0.15
    max_lateral_error_m: float = 1.5
    progress_window: int = 100
    progress_min_gain_m: float = 0.1


def check_termination(
    cfg: TerminationConfig,
    step: int,
    arc: float,
    arc_history: list,
    e_y: float,
    total_length: float,
    health: HealthState,
) -> tuple:
    """Return ``(terminated, reason)``. Deterministic."""
    if step >= cfg.max_steps:
        return True, "max_steps"
    if arc >= total_length - cfg.complete_margin_m:
        return True, "completed"
    if abs(e_y) > cfg.max_lateral_error_m:
        return True, "out_of_track"
    if health == HealthState.EMERGENCY_STOP:
        return True, "emergency_stop"
    if health.is_critical:
        return True, "controller_fault"
    if len(arc_history) >= cfg.progress_window:
        gained = arc - arc_history[-cfg.progress_window]
        if gained < cfg.progress_min_gain_m:
            return True, "stall"
    return False, ""
