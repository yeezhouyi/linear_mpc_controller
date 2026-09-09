"""Tracker-bound reference production entry (advanced round, 必做1).

The fixable half of the guard semantics, composed into ONE entry point
that every reference takes before it may reach a controller:

    1. resample_uniform(x, y, ds)         -- arc-uniform samples (the
       curvature estimator is only meaningful on near-uniform input);
    2. complete_speed_curvature(...)      -- tangent/curvature + kinematic
       omega cap (|kappa| * v <= omega_max), floor inside the legal band;
    3. accel_limited_profile(v, s, ...)   -- forward accel ramp + terminal
       brake to v_end (default 0 -> terminal deceleration);
    4. certify_reference(...)             -- the single gate.  A returned
       verdict with feasible=False must go back to the planner/task layer
       with the stable code; it is NEVER handed to a controller.

This is the adapter "fix pipeline" that run_feasibility_matrix calls
``guarded``; keeping it here as a library function (instead of only
inside a benchmark script) gives production code one importable entry
and keeps ``certify_reference`` as the real last check before a
controller is invoked.
"""
from __future__ import annotations

import numpy as np

from mpc_core.types import Trajectory

from trajectory_tools.curvature_estimator import (
    complete_speed_curvature,
)
from trajectory_tools.reference_guard import certify_reference
from trajectory_tools.resample import resample_uniform
from trajectory_tools.speed_profile import (
    DEFAULT_A_ACCEL,
    DEFAULT_A_DECEL,
    accel_limited_profile,
    speed_profile_violations,
)

DEFAULT_V_DEFAULT = 0.3   # cruise speed the adapter asks for
DEFAULT_V_MAX = 0.5       # hard ceiling (vehicle/battery or task limit)
DEFAULT_DS = 0.05         # arc resample step, m


def prepare_tracker_reference(
    x: np.ndarray,
    y: np.ndarray,
    v_default: float = DEFAULT_V_DEFAULT,
    v_max: float = DEFAULT_V_MAX,
    ds: float = DEFAULT_DS,
    omega_max: float = 2.0,
    a_accel: float = DEFAULT_A_ACCEL,
    a_decel: float = DEFAULT_A_DECEL,
    v_end: float = 0.0,
) -> tuple:
    """Full adapter fix pipeline + certification for a tracker-bound reference.

    Returns ``(trajectory, verdict)`` where ``verdict`` is the
    ``certify_reference`` dict (feasible / codes / reasons / path /
    motion).  Callers MUST check ``verdict["feasible"]`` before handing
    ``trajectory`` to a controller; an infeasible verdict carries the
    reason code for the planner/task layer.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.size < 2 or y.size < 2:
        raise ValueError(
            "prepare_tracker_reference needs >= 2 waypoints "
            "(got x=%d, y=%d)" % (x.size, y.size))

    xr, yr = resample_uniform(x, y, ds=ds)
    yaw, kappa, v = complete_speed_curvature(xr, yr, v_default, v_max,
                                             omega_max=omega_max)
    n = xr.size
    s = np.zeros(n)
    if n > 1:
        s[1:] = np.cumsum(np.hypot(np.diff(xr), np.diff(yr)))
    vp = accel_limited_profile(v, s, a_accel=a_accel, a_decel=a_decel,
                               v_end=v_end)
    traj = Trajectory(s=s, x=xr, y=yr, yaw=yaw, kappa=kappa, v=vp,
                      segment_gear=np.ones(max(1, n - 1)))
    verdict = certify_reference(traj.x, traj.y, traj.kappa, traj.v,
                                traj.s, yaw=traj.yaw, omega_max=omega_max)
    # Belt & braces: the profile check is part of the processed reference's
    # "satisfies the discrete constraint checks" acceptance.
    prof = speed_profile_violations(vp, s, a_accel=a_accel,
                                    a_decel=a_decel, v_end=v_end)
    if prof["codes"] and not verdict["feasible"]:
        verdict = dict(verdict)
        verdict["reasons"] = list(verdict["reasons"]) + list(prof["reasons"])
        verdict["codes"] = list(verdict["codes"]) + list(prof["codes"])
    return traj, verdict
