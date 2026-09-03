"""Frenet projection & error convention tests."""
import math

import numpy as np
import pytest

from mpc_core.frenet import closest_point, frenet_state
from mpc_core.types import KinematicState, Trajectory, wrap_angle
from trajectory_tools.reference_trajectory import make_circle, make_straight


def test_straight_lateral_sign_and_value():
    traj = make_straight(length=8.0)  # along +x, yaw=0
    # robot above the line (left of travel +x -> y>0 means left)
    st = KinematicState(x=2.0, y=0.4, yaw=0.0, v=0.8, omega=0.0)
    anchor, err = frenet_state(traj, st)
    assert anchor.s == pytest.approx(2.0, abs=1e-6)
    assert err[0] == pytest.approx(0.4, abs=1e-9)  # e_y = +y (left positive)
    assert err[1] == pytest.approx(0.0, abs=1e-9)

    st2 = KinematicState(x=2.0, y=-0.3, yaw=0.0, v=0.8, omega=0.0)
    _, err2 = frenet_state(traj, st2)
    assert err2[0] == pytest.approx(-0.3, abs=1e-9)


def test_circle_left_positive_towards_centre():
    """make_circle(radius=2) starts at (0,0) heading +x and turns CCW, so the
    circle centre sits at (0, +R) on the robot's LEFT.  e_y > 0 = left."""
    traj = make_circle(radius=2.0)
    # inside (towards centre, left of travel at the bottom point) -> +0.2
    st = KinematicState(x=0.0, y=0.2, yaw=0.0, v=0.6, omega=0.3)
    anchor, err = frenet_state(traj, st)
    assert err[0] == pytest.approx(0.2, abs=0.02)
    # outside (right of travel) -> -0.2
    st2 = KinematicState(x=0.0, y=-0.2, yaw=0.0, v=0.6, omega=0.3)
    _, err2 = frenet_state(traj, st2)
    assert err2[0] == pytest.approx(-0.2, abs=0.02)


def test_heading_error_wrap():
    traj = make_straight()
    st = KinematicState(x=1.0, y=0.0, yaw=3.0, v=0.8, omega=0.0)
    _, err = frenet_state(traj, st)
    assert err[1] == pytest.approx(wrap_angle(3.0), abs=1e-9)


def test_heading_error_on_circle():
    # CCW circle radius 2 centred at (0, 2): the point at angle 0 is (2, 2)
    # and its tangent heading is +pi/2 (north).
    traj = make_circle(radius=2.0, ds=0.01)
    st = KinematicState(x=2.0, y=2.0, yaw=math.pi / 2.0 + 0.1, v=0.6, omega=0.3)
    _, err = frenet_state(traj, st)
    assert err[0] == pytest.approx(0.0, abs=0.02)
    assert err[1] == pytest.approx(0.1, abs=0.01)


def test_clamp_at_ends():
    traj = make_straight(length=4.0)
    # beyond the end: anchor clamps to the last point
    st = KinematicState(x=10.0, y=0.2, yaw=0.0, v=0.8, omega=0.0)
    anchor, err = frenet_state(traj, st)
    assert anchor.s == pytest.approx(traj.s[-1], abs=1e-6)
    # lateral error still measured against the terminal segment tangent
    assert abs(err[0]) >= 0.199


def test_lookahead_moves_anchor_forward():
    traj = make_straight(length=8.0)
    st = KinematicState(x=2.0, y=0.0, yaw=0.0, v=0.8, omega=0.0)
    anchor0, _ = frenet_state(traj, st, lookahead_m=0.0)
    anchor1, _ = frenet_state(traj, st, lookahead_m=0.6)
    assert anchor0.s == pytest.approx(2.0, abs=1e-9)
    assert anchor1.s == pytest.approx(2.6, abs=1e-9)


def test_closest_point_deterministic_with_duplicates():
    traj = make_straight()
    # add a duplicate vertex by rebuilding arrays manually
    tr = Trajectory(
        s=np.array([0.0, 0.0, 1.0, 2.0, 3.0]),
        x=np.array([0.0, 0.0, 1.0, 2.0, 3.0]),
        y=np.zeros(5),
        yaw=np.zeros(5),
        kappa=np.zeros(5),
        v=np.full(5, 0.8),
    )
    seg, w, e_y, arc = closest_point(tr, 1.4, 0.05)
    assert e_y == pytest.approx(0.05, abs=1e-9)
    assert arc == pytest.approx(1.4, abs=1e-6)
    # run twice, same answer
    assert (seg, w, e_y, arc) == closest_point(tr, 1.4, 0.05)
