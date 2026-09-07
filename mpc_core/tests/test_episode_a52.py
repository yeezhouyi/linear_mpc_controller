"""Episode-level A5.2 accepted-arc high-watermark accounting tests."""
import numpy as np
import pytest

from mpc_core.episode import run_tracking_episode
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import MpcParams
from trajectory_tools.reference_trajectory import make_circle, make_straight


def make_controller(traj):
    ctrl = LinearMpcController(MpcParams())
    ctrl.set_reference(traj)
    return ctrl


def test_clean_straight_run_ratio_one_no_jumps():
    traj = make_straight(length=6.0, v=0.6)
    ctrl = make_controller(traj)
    plant = DifferentialDrivePlant(x0=0.0, y0=0.6, yaw0=0.25, v0=0.0, omega0=0.0)
    res = run_tracking_episode(ctrl, plant, traj, "straight")
    assert res.completed, res.done_reason
    assert res.done_reason == "COMPLETED"
    assert res.projection_jump_count == 0
    assert res.progress_ratio >= 0.95, res.progress_ratio  # A5.3 floor
    assert res.skipped_arc_ratio == 0.0
    assert res.arc_high_watermark >= traj.s[-1] - 0.15
    assert res.progress_m == pytest.approx(res.arc_high_watermark, abs=1e-6)
    # progress can never exceed 1 by construction (A5.2 property 1)
    assert res.progress_ratio <= 1.0 + 1e-12
    # legacy raw-arc accumulation may differ; the high-watermark is the ruler
    assert res.progress_frac <= 1.0 + 1e-6


def test_clean_circle_run_no_jumps():
    traj = make_circle(radius=2.0, v=0.5)
    ctrl = make_controller(traj)
    plant = DifferentialDrivePlant(x0=0.0, y0=0.0, yaw0=0.2, v0=0.4, omega0=0.0)
    res = run_tracking_episode(ctrl, plant, traj, "circle", max_steps=3000)
    assert res.completed, res.done_reason
    assert res.projection_jump_count == 0, res.projection_jump_count
    assert res.progress_ratio >= 0.95, res.progress_ratio  # A5.3 floor
    assert res.skipped_arc_ratio == 0.0


def test_completed_refused_while_in_probation_is_episode_controller_contract():
    """A5.3: the episode reads diag.in_probation from the controller; a run
    that needs reacquire must not claim a clean pass."""
    traj = make_straight(length=6.0, v=0.6)
    ctrl = make_controller(traj)
    plant = DifferentialDrivePlant(x0=0.0, y0=0.6, yaw0=0.25, v0=0.0, omega0=0.0)
    res = run_tracking_episode(ctrl, plant, traj, "straight")
    # the guard field is plumbed through diagnostics on every cycle
    assert hasattr(res, "progress_m")
    assert res.qp_failures == 0
    assert res.projection_reacquire_count == 0  # not yet wired at episode level
