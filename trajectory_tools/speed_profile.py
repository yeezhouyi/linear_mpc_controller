"""Along-path speed profile builder (advanced round, 必做1 close-out).

The kinematic cap from ``complete_speed_curvature`` bounds ``|kappa|*v``
but says nothing about HOW FAST the reference speed may CHANGE along the
path.  A step from 0.30 m/s to 0.50 m/s over one 0.05 m sample implies
an absurd longitudinal acceleration, and a reference that stays at
cruise speed until the very last sample never tells the tracker to stop.

This module builds the speed half of a tracker-bound reference so that
the DISCRETE along-path checks hold:

  * forward ramp : v[i]^2 - v[i-1]^2 <= 2 * a_accel * ds
                   (kinetic energy may grow at most linearly with arc);
  * terminal brake: v[i]^2 - v[i+1]^2 <= 2 * a_decel * ds computed as a
                   backward pass from ``v_end``, so the reference can
                   actually come to ``v_end`` by the path end (default
                   v_end = 0 -> terminal deceleration);
  * never raises a sample above the input curve (the curvature/omega
    cap from the caller is preserved).

The builder is the "fixable" half of the guard semantics (resample /
speed-shape = repairable).  ``certify_reference`` remains the single
gate that VERIFIES the result; this module never decides feasibility.
"""
from __future__ import annotations

import math

import numpy as np

DEFAULT_A_ACCEL = 0.5   # m/s^2, longitudinal accel limit (cruise build-up)
DEFAULT_A_DECEL = 0.5   # m/s^2, longitudinal decel limit (terminal brake)


def accel_limited_profile(
    v_curve: np.ndarray,
    s: np.ndarray,
    a_accel: float = DEFAULT_A_ACCEL,
    a_decel: float = DEFAULT_A_DECEL,
    v_end: float = 0.0,
) -> np.ndarray:
    """Shape ``v_curve`` into an along-path feasible speed profile.

    Passes, in order:
      1. forward ramp  -- no sample may exceed the energy its predecessor
         can reach under ``a_accel`` over the intervening arc;
      2. terminal brake -- backward pass from ``v_end`` so every sample
         can still brake to ``v_end`` by ``s[-1]`` under ``a_decel``.

    Both passes only LOWER samples (``min`` against the current value),
    so any omega cap already enforced in ``v_curve`` is preserved.
    Constant-arc samples (ds <= 0) are left untouched.
    """
    v = np.asarray(v_curve, dtype=float).copy()
    s_arr = np.asarray(s, dtype=float)
    n = v.size
    if n == 0:
        return v

    def _bound(lim2: float) -> float:
        return math.sqrt(max(0.0, lim2))

    # 1) forward ramp: v[i]^2 <= v[i-1]^2 + 2*a_accel*ds
    for i in range(1, n):
        ds = s_arr[i] - s_arr[i - 1]
        if ds > 0.0:
            lim2 = v[i - 1] ** 2 + 2.0 * a_accel * ds
            if v[i] ** 2 > lim2:
                v[i] = _bound(lim2)

    # 2) terminal brake: v[i]^2 <= v[i+1]^2 + 2*a_decel*ds, v[-1] <= v_end
    if n > 0 and v[-1] > float(v_end):
        v[-1] = float(v_end)
    for i in range(n - 2, -1, -1):
        ds = s_arr[i + 1] - s_arr[i]
        if ds > 0.0:
            lim2 = v[i + 1] ** 2 + 2.0 * a_decel * ds
            if v[i] ** 2 > lim2:
                v[i] = _bound(lim2)

    # numeric safety only: the builder never creates negative speeds
    return np.maximum(v, 0.0)


def speed_profile_violations(
    v: np.ndarray,
    s: np.ndarray,
    a_accel: float = DEFAULT_A_ACCEL,
    a_decel: float = DEFAULT_A_DECEL,
    v_end: float = 0.0,
    ds_min: float = 1e-9,
    tol: float = 1e-6,
) -> dict:
    """Discrete along-path constraint check for a candidate speed profile.

    Verifies, per adjacent sample pair (over an arc step > ``ds_min``):
      accel_bound -- v[i]^2 - v[i-1]^2 <= 2*a_accel*ds (+ tol);
      brake_bound -- v[i]^2 - v[i+1]^2 <= 2*a_decel*ds (+ tol).
    Plus scalar checks:
      negative_speed / terminal_speed (v[-1] <= v_end) / nonfinite.

    Returns a dict with ``feasible``, human ``reasons`` and stable
    ``codes`` (nonfinite / negative_speed / accel_bound / brake_bound /
    terminal_speed).
    """
    v = np.asarray(v, dtype=float)
    s = np.asarray(s, dtype=float)
    n = min(v.size, s.size)
    reasons: list[str] = []
    codes: list[str] = []

    if n == 0:
        return {"feasible": True, "codes": [], "reasons": [], "indices": []}

    nf = (~np.isfinite(v[:n])).nonzero()[0]
    if nf.size:
        codes.append("nonfinite")
        reasons.append(
            "nonfinite: %d speed sample(s) at %s (code=nonfinite)"
            % (nf.size, nf[:8].tolist()))
    if np.any(v[:n] < -tol):
        codes.append("negative_speed")
        reasons.append(
            "negative_speed: profile drops below 0 (builder invariant "
            "broken) (code=negative_speed)")

    idx: list[int] = []
    for i in range(1, n):
        ds = s[i] - s[i - 1]
        if ds <= ds_min:
            continue
        tol_sq = tol * max(1.0, ds)
        dv2 = v[i] ** 2 - v[i - 1] ** 2
        if dv2 > 2.0 * a_accel * ds + tol_sq:
            codes.append("accel_bound")
            idx.append(i)
            reasons.append(
                "accel_bound: sample %d ramps faster than a_accel=%.3f "
                "(code=accel_bound)" % (i, a_accel))
            break
    for i in range(n - 1):
        ds = s[i + 1] - s[i]
        if ds <= ds_min:
            continue
        tol_sq = tol * max(1.0, ds)
        dv2 = v[i] ** 2 - v[i + 1] ** 2
        if dv2 > 2.0 * a_decel * ds + tol_sq:
            codes.append("brake_bound")
            idx.append(i)
            reasons.append(
                "brake_bound: sample %d cannot brake to its successor "
                "under a_decel=%.3f (code=brake_bound)" % (i, a_decel))
            break
    if n > 0 and v[n - 1] > float(v_end) + tol:
        codes.append("terminal_speed")
        reasons.append(
            "terminal_speed: last sample %.4f > v_end=%.3f "
            "(code=terminal_speed)" % (v[n - 1], v_end))

    # de-duplicate while preserving order of first appearance
    seen: set[str] = set()
    codes_uniq = [c for c in codes if not (c in seen or seen.add(c))]
    return {
        "feasible": not codes_uniq,
        "codes": codes_uniq,
        "reasons": reasons,
        "indices": sorted(set(idx)),
    }
