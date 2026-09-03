"""Closed-loop linear-MPC tracking tests (AE1 / U3 verification)."""
import numpy as np
import pytest

from mpc_core.episode import run_tracking_episode
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, MpcParams
from trajectory_tools.reference_trajectory import make_circle, make_s_curve, make_straight, make_u_turn


def make_controller(traj, **kw):
    defaults = dict(Ts=0.05, N=25, Q_diag=(120.0, 25.0, 3.0, 1.5),
                    Q_F_diag=(200.0, 40.0, 5.0, 3.0), S_diag=(0.4, 0.4),
                    qp_max_iter=1200, qp_rel_tol=1e-4, qp_abs_tol=1e-6,
                    lookahead_m=0.0)
    defaults.update(kw)
    params = MpcParams(**defaults)
    return LinearMpcController(params, traj)


def test_straight_line_converges():
    traj = make_straight(length=6.0, v=0.6, ds=0.01)
    # start offset laterally + heading error
    plant = DifferentialDrivePlant(x0=0.5, y0=0.6, yaw0=0.25, v0=0.0, omega0=0.0)
    ctrl = make_controller(traj)
    res = run_tracking_episode(ctrl, plant, traj, "straight", verbose=True)
    assert res.completed, res.done_reason
    # transient-dominated RMS for this offset start; steady state must be ~0
    assert res.e_y_rms < 0.30
    assert res.e_y_p95 < 0.65
    assert np.mean(np.abs(res.e_y[-80:])) < 0.03
    assert res.qp_failures == 0
    assert max(res.constraint_violation, default=0.0) < 1e-4


def test_circle_tracking_bounds_respected():
    traj = make_circle(radius=2.0, v=0.5, ds=0.01)
    plant = DifferentialDrivePlant(x0=0.0, y0=0.0, yaw0=0.2, v0=0.4, omega0=0.0)
    ctrl = make_controller(traj)
    res = run_tracking_episode(ctrl, plant, traj, "circle", verbose=True)
    assert res.completed, res.done_reason
    assert res.e_y_rms < 0.05
    assert res.e_y_p95 < 0.10
    assert res.qp_failures == 0
    # velocity never exceeds v_max + tiny slack
    assert max(res.v) <= ctrl.params.v_max + 1e-3
    assert max(res.omega) <= ctrl.params.omega_max + 1e-3
    assert max(res.constraint_violation, default=0.0) < 1e-3


def test_u_turn_and_s_curve_complete():
    for name, maker in (("s_curve", make_s_curve), ("u_turn", make_u_turn)):
        traj = maker(ds=0.01)
        plant = DifferentialDrivePlant(x0=traj.x[0] - 0.4, y0=traj.y[0] + 0.3,
                                       yaw0=traj.yaw[0] + 0.2, v0=0.0, omega0=0.0)
        ctrl = make_controller(traj)
        res = run_tracking_episode(ctrl, plant, traj, name, verbose=True, max_steps=6000)
        assert res.completed, f"{name}: {res.done_reason}"
        assert res.e_y_rms < 0.20, f"{name}: e_y_rms={res.e_y_rms:.3f}"
        assert res.qp_failures == 0, name


def test_no_reference_returns_zero():
    ctrl = LinearMpcController(MpcParams())
    from mpc_core.types import KinematicState

    out = ctrl.compute_cycle(KinematicState(x=0.0, y=0.0, yaw=0.0, v=0.5, omega=0.0))
    assert out.v_cmd == 0.0 and out.omega_cmd == 0.0
    assert out.diag.health == HealthState.NO_REFERENCE


def test_solver_speed_budget():
    """QP solve time must stay well below the 20 Hz budget in the reference
    core (informational: the realtime gate runs on the C++/OSQP core)."""
    traj = make_straight(length=4.0, v=0.6)
    ctrl = make_controller(traj)
    plant = DifferentialDrivePlant(x0=0.3, y0=0.2, yaw0=0.1, v0=0.2, omega0=0.0)
    res = run_tracking_episode(ctrl, plant, traj, "timing", max_steps=120)
    times = np.array(res.qp_time_us)
    assert np.median(times) < 40_000  # reference core on a laptop: <<50 ms budget
