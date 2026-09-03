"""Tracking metrics (R22 / plan metrics definitions).

Computes the formal per-run metrics from an :class:`EpisodeResult`:

* lateral error  ``e_y``: signed Frenet lateral distance (RMSE / p95 / max),
* heading error  ``e_psi``: wrapped angle error in [-pi, pi),
* speed error    ``e_v = v - v_ref``,
* control smoothness ``J_smooth = mean ||u_t - u_{t-1}||^2``,
* QP solver timing (mean / p95 / max), iteration and status counts,
* hard-constraint violations, fallback and QP-failure counts.
"""
from __future__ import annotations

from typing import Dict

import numpy as np

from mpc_core.episode import EpisodeResult
from mpc_core.types import HealthState


def _p95(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.percentile(np.abs(x), 95)) if x.size else float("nan")


def _rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(np.square(x)))) if x.size else float("nan")


def compute_tracking_metrics(res: EpisodeResult, v_ref: float = 0.0) -> Dict:
    e_y = np.asarray(res.e_y, dtype=float)
    e_psi = np.asarray(res.e_psi, dtype=float)
    v = np.asarray(res.v, dtype=float)
    cmd_v = np.asarray(res.cmd_v, dtype=float)
    cmd_w = np.asarray(res.cmd_w, dtype=float)
    qp_t = np.asarray(res.qp_time_us, dtype=float)
    qp_it = np.asarray(res.qp_iterations, dtype=int)

    # control change rate per cycle (J_smooth, plan metrics definitions)
    du = np.concatenate([np.diff(cmd_v), np.diff(cmd_w)])
    j_smooth = float(np.mean(np.square(du))) if du.size else 0.0

    e_v = v - v_ref
    return {
        "track": res.track_name,
        "completed": bool(res.completed),
        "done_reason": res.done_reason,
        "steps": int(res.steps),
        "t_end_s": round(float(res.t_end), 3),
        "e_y_rms": round(_rms(e_y), 5),
        "e_y_p95": round(_p95(e_y), 5),
        "e_y_max": round(float(np.max(np.abs(e_y))) if e_y.size else float("nan"), 5),
        "e_psi_rms": round(_rms(e_psi), 5),
        "e_psi_p95": round(_p95(e_psi), 5),
        "e_v_rms": round(_rms(e_v), 5),
        "e_v_p95": round(_p95(e_v), 5),
        "j_smooth": round(j_smooth, 6),
        "qp_time_us_mean": round(float(np.mean(qp_t)), 1) if qp_t.size else 0.0,
        "qp_time_us_p95": round(float(np.percentile(qp_t, 95)), 1) if qp_t.size else 0.0,
        "qp_time_us_max": round(float(np.max(qp_t)), 1) if qp_t.size else 0.0,
        "qp_iter_mean": round(float(np.mean(qp_it)), 1) if qp_it.size else 0,
        "qp_status_counts": {
            s: int((np.asarray(res.qp_status) == s).sum()) for s in set(res.qp_status)
        },
        "qp_failures": int(res.qp_failures),
        "fallback_count": int(res.fallback_count),
        "constraint_violation_max": round(
            float(np.max(res.constraint_violation)) if res.constraint_violation else 0.0, 6
        ),
        "emergency_stops": int(
            np.asarray(res.health, dtype=int).tolist().count(int(HealthState.EMERGENCY_STOP))
        ) if res.health else 0,
    }


def format_metrics_md(metrics: Dict) -> str:
    """Compact markdown table row for a metrics dict."""
    return (
        f"| {metrics['track']} | {metrics['done_reason']} | {metrics['steps']} | "
        f"{metrics['e_y_rms']:.3f} | {metrics['e_y_p95']:.3f} | {metrics['e_y_max']:.3f} | "
        f"{metrics['e_psi_rms']:.3f} | {metrics['e_v_rms']:.3f} | "
        f"{metrics['qp_time_us_mean']:.0f} | {metrics['qp_time_us_p95']:.0f} | "
        f"{metrics['qp_failures']} | {metrics['fallback_count']} | "
        f"{metrics['constraint_violation_max']:.1e} |"
    )
