"""Curvature estimator tests (trajectory adapter completion rule, Day 4-5).

Day 4-5 scope change: completion is now kinematically bounded by omega_max
(|kappa|*v <= omega_max), and the completion floor only applies where it
cannot violate that bound.  The old ``max(floor, cap)`` shape pushed the
already-correct cap back above omega_max on sharp points; the red-side
guard test below pins that exact failure so it cannot return.
"""
import numpy as np
import pytest

from trajectory_tools.curvature_estimator import (
    complete_speed_curvature,
    estimate_curvature,
    estimate_heading,
    omega_violations,
)
from trajectory_tools.reference_trajectory import make_circle, make_s_curve


def test_circle_curvature_recovered():
    traj = make_circle(radius=2.0, ds=0.01)
    kappa = estimate_curvature(traj.x, traj.y)
    # interior points only (endpoints copy neighbours)
    mid = slice(10, -10)
    assert np.median(kappa[mid]) == pytest.approx(0.5, abs=0.02)


def test_heading_matches_tangent():
    traj = make_circle(radius=2.0, ds=0.01)
    yaw = estimate_heading(traj.x, traj.y)
    mid = slice(10, -10)
    err = np.abs((yaw[mid] - traj.yaw[mid] + np.pi) % (2 * np.pi) - np.pi)
    assert np.max(err) < 0.02


def test_s_curve_sign_change():
    traj = make_s_curve(ds=0.01)
    kappa = estimate_curvature(traj.x, traj.y)
    # the S track has a positive arc followed by a negative arc
    assert np.max(kappa) > 0.3
    assert np.min(kappa) < -0.3


def test_completion_rule_deterministic_and_bounded():
    traj = make_circle(radius=1.0, ds=0.02)
    yaw1, kappa1, v1 = complete_speed_curvature(
        traj.x, traj.y, v_default=0.6, v_max=1.5)
    yaw2, kappa2, v2 = complete_speed_curvature(
        traj.x, traj.y, v_default=0.6, v_max=1.5)
    assert np.array_equal(v1, v2)          # deterministic
    assert np.max(v1) <= 1.5 + 1e-12       # bounded by v_max
    # gentle circle (R=1, kappa=1): omega cap 2.0/1 = 2.0 > default 0.6,
    # so the default stands and omega = kappa*v <= 2.0 holds
    assert np.max(v1) <= 0.6 + 1e-9
    viol, _ = omega_violations(kappa1, v1, omega_max=2.0)
    assert viol.size == 0
    assert np.array_equal(yaw1, yaw2) and np.array_equal(kappa1, kappa2)


def test_completion_sharp_corner_never_raises_above_omega():
    """Day 4-5 core fix: on a sharp corner the floor must NOT push v back
    above the omega bound (old rule gave v=0.15 at |kappa|=20 -> 3.0 rad/s
    vs bound 2.0; new rule yields v = 2.0/|kappa| = 0.1 there)."""
    traj = make_circle(radius=0.05, ds=0.0025)   # kappa ~ 20 (R = 0.05 m)
    _, kappa, v = complete_speed_curvature(
        traj.x, traj.y, v_default=0.5, v_max=0.5, omega_max=2.0, v_floor=0.15)
    viol, ratio = omega_violations(kappa, v, omega_max=2.0)
    assert viol.size == 0, f"{viol.size} pts violate omega (max ratio {ratio})"
    # sharp points must actually be below the floor (no raise)
    k_abs = np.abs(kappa)
    sharp = k_abs > 2.0 / 0.15        # outside the legal floor band
    assert sharp.any()
    assert float(np.max(v[sharp])) < 0.15 + 1e-9


def test_guard_red_side_old_floor_shape():
    """Red side pinned: the OLD floor-after-cap shape (v=0.15 at
    |kappa|=20) violates omega_max -- this is the regression the guard
    exists to catch."""
    kappa = np.array([20.0])
    v_old = np.array([0.15])          # old rule output on that point
    viol, ratio = omega_violations(kappa, v_old, omega_max=2.0)
    assert viol.size == 1
    assert ratio == pytest.approx(1.5)   # 20*0.15 / 2.0
