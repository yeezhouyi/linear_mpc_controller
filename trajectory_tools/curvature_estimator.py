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

    NOTE (Day 4-5): this is a finite difference and is only meaningful on
    (near-)uniformly spaced input.  Resample by arc length first
    (trajectory_tools.resample.resample_uniform) -- on the raw non-uniform
    u9 plan it manufactures phantom corners (R~0.03 m at folds).
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
                             v_max: float, omega_max: float = 2.0,
                             v_floor: float = 0.15) -> tuple:
    """Adapter completion rule: fill tangent/curvature/speed from poses only.

    Deterministic rule: default forward speed, capped so that the implied
    angular rate stays inside the motion limit, ``|kappa| * v <= omega_max``:

        v = min(v_default, omega_max / |kappa|)   (where |kappa| > 0)

    The completion FLOOR (kept for its original purpose -- preventing the
    tracker crawling at recorded sharp corners) is applied ONLY where it
    cannot violate the omega bound, i.e. on points with
    ``|kappa| <= omega_max / v_floor``.  On tighter points the pure
    kinematic cap stands (no raise): this is the Day 4-5 fix -- the old
    ``max(floor, cap)`` pushed an already-correct cap back ABOVE the motion
    limit (v=0.15 at |kappa|=31.4 requires 4.7 rad/s > 2.0).

    Returns ``(yaw, kappa, v)`` arrays.
    """
    yaw = estimate_heading(x, y)
    kappa = estimate_curvature(x, y)
    k_abs = np.abs(kappa)
    eps = 1e-6
    v = np.full(len(x), float(v_default))
    # kinematic cap: |kappa| * v <= omega_max
    with np.errstate(divide="ignore", invalid="ignore"):
        cap = np.where(k_abs > eps, omega_max / np.maximum(k_abs, eps),
                       float(v_default))
    v = np.minimum(v, cap)
    # floor only inside the legal band (|kappa| <= omega_max / v_floor)
    legal = k_abs <= omega_max / max(float(v_floor), 1e-9)
    v = np.where(legal, np.maximum(v, float(v_floor)), v)
    v = np.minimum(v, float(v_max))
    return yaw, kappa, v


def omega_violations(kappa: np.ndarray, v: np.ndarray,
                     omega_max: float = 2.0) -> tuple:
    """Guard (Day 4-5): reference feasibility ``max(|kappa| * |v|) <= omega_max``.

    Returns ``(indices, max_ratio)`` where index i is violating when
    ``|kappa[i]| * |v[i]| > omega_max * (1 + 1e-9)`` and ``max_ratio`` is the
    largest ``|kappa|*|v|/omega_max`` over all points (0.0 when no data).
    A reference trajectory entering a tracker MUST satisfy this; the old
    floor-after-cap rule violated it on 16/250 points of the real u9 plan
    (max 4.71 rad/s vs 2.0).
    """
    kappa = np.asarray(kappa, dtype=float)
    v = np.asarray(v, dtype=float)
    n = min(kappa.size, v.size)
    if n == 0:
        return (np.array([], dtype=int), 0.0)
    omega = np.abs(kappa[:n]) * np.abs(v[:n])
    bound = omega_max * (1.0 + 1e-9)
    idx = np.nonzero(omega > bound)[0]
    ratio = float(omega.max() / omega_max) if omega.size else 0.0
    return (idx, ratio)


def math_atan2(dy: float, dx: float) -> float:
    import math

    return math.atan2(dy, dx)


def math_hypot(a: float, b: float) -> float:
    import math

    return math.hypot(a, b)
