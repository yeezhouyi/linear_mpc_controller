"""Uniform arc-length resampling (Day 4-5, delivery-denominator doc 1.3).

The real u9 planner output is NON-uniform (point spacing 0.050 ~ 4.550 m,
30 gaps > 0.5 m).  Finite-difference curvature on such a polyline produces
phantom corners (R ~ 0.03 m at 90-degree folds) and a speed profile that
is then wrong everywhere.  Fix order is rigid: 1) resample uniformly by
arc length, 2) recompute curvature on the resampled points, 3) only then
apply any speed floor (and only where it cannot violate the omega bound).
"""
from __future__ import annotations

import numpy as np


def resample_uniform(
    x,
    y,
    ds: float = 0.05,
) -> tuple:
    """Resample a polyline to near-uniform arc-length spacing ``ds``.

    Cumulative chord length is used as the parameter; the output keeps the
    two endpoints exactly and inserts points at ``ds`` spacing (the final
    interval takes the remainder).  Returns ``(xr, yr)`` as float arrays.
    Degenerate (zero-length / <2 points) input is returned unchanged.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = x.size
    if n < 2 or not (ds > 0.0):
        return x.copy(), y.copy()
    dx = np.diff(x)
    dy = np.diff(y)
    seg = np.hypot(dx, dy)
    total = float(seg.sum())
    if total <= 0.0 or not np.isfinite(total):
        return x.copy(), y.copy()
    arc = np.concatenate(([0.0], np.cumsum(seg)))
    m = max(2, int(round(total / ds)) + 1)
    t = np.linspace(0.0, total, m)
    xr = np.interp(t, arc, x)
    yr = np.interp(t, arc, y)
    return xr, yr
