"""Numeric curvature / tangent estimator (U2, trajectory adapter use-case).

Estimates per-point tangent heading and signed curvature from dense world
positions only -- this is what a ROS2 adapter does when upstream paths carry
no velocity/curvature (R6 completion rule).

Convention matches the rest of the repo: ``kappa > 0`` = left turn.
"""
from __future__ import annotations

import numpy as np

from mpc_core.types import wrap_angle


def estimate_heading(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Tangent heading per point from neighbour differences."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    yaw = np.zeros(n)
    for i in range(n):
        j0 = max(i - 1, 0)
        j1 = min(i + 1, n - 1)
        dx = x[j1] - x[j0]
        dy = y[j1] - y[j0]
        if j1 == j0:
            dx, dy = 1.0, 0.0
        yaw[i] = math_atan2(dy, dx)
    return yaw


def estimate_curvature(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Signed curvature per point using heading differences over arc length.

    ``kappa[i] = wrap(yaw[i+1] - yaw[i-1]) / (2*ds_i)`` where the arc step is
    the chord distance.  Endpoints copy their neighbour.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    yaw = estimate_heading(x, y)
    kappa = np.zeros(n)
    if n < 3:
        return kappa
    for i in range(1, n - 1):
        d_prev = math_hypot(x[i] - x[i - 1], y[i] - y[i - 1])
        d_next = math_hypot(x[i + 1] - x[i], y[i + 1] - y[i])
        arc = 0.5 * (d_prev + d_next)
        if arc > 1e-12:
            kappa[i] = wrap_angle(yaw[i + 1] - yaw[i - 1]) / (2.0 * arc)
    kappa[0] = kappa[1] if n > 1 else 0.0
    kappa[-1] = kappa[-2] if n > 1 else 0.0
    return kappa


def complete_speed_curvature(x: np.ndarray, y: np.ndarray, v_default: float,
                             v_max: float) -> tuple:
    """Adapter completion rule: fill tangent/curvature/speed from poses only.

    Deterministic rule (documented in docs/ros2_interface_contract.md):
    default forward speed, capped so that ``|omega| = |kappa| * v <= omega_max``
    when an angular-velocity bound is given via ``v_max`` semantics -- here the
    cap keeps curvature * curvature * v within a mild bound.  Returns
    ``(yaw, kappa, v)`` arrays.
    """
    yaw = estimate_heading(x, y)
    kappa = estimate_curvature(x, y)
    # gentle speed shaping in curves (deterministic, monotone)
    k_abs = np.abs(kappa)
    v = np.full(len(x), float(v_default))
    cap = np.where(k_abs > 1e-6, min(v_max, 0.6) / np.maximum(k_abs, 1e-6), v_default)
    v = np.minimum(v, np.maximum(cap, 0.1))
    v = np.minimum(v, v_max)
    return yaw, kappa, v


def math_atan2(dy: float, dx: float) -> float:
    import math

    return math.atan2(dy, dx)


def math_hypot(a: float, b: float) -> float:
    import math

    return math.hypot(a, b)
