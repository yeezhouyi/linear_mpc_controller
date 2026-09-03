"""Curvature estimator tests (trajectory adapter completion rule)."""
import numpy as np
import pytest

from trajectory_tools.curvature_estimator import complete_speed_curvature, estimate_curvature, estimate_heading
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
    yaw1, kappa1, v1 = complete_speed_curvature(traj.x, traj.y, v_default=0.6, v_max=1.5)
    yaw2, kappa2, v2 = complete_speed_curvature(traj.x, traj.y, v_default=0.6, v_max=1.5)
    assert np.array_equal(v1, v2)          # deterministic
    assert np.max(v1) <= 1.5 + 1e-12       # bounded by v_max
    # tight curve (R=1, kappa=1): speed capped by curve_speed/kappa = 0.6
    assert np.max(v1) <= 0.6 + 1e-9
    assert np.array_equal(yaw1, yaw2) and np.array_equal(kappa1, kappa2)
