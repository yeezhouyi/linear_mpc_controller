"""A5.1 acceptance-gate behaviour tests at the CONTROLLER level.

The gate lives in the control chain (mpc.py): the QP anchor is taken at the
ACCEPTED arc only (A5.0).  Behaviour pins (doc A5.1/A7.2 + A4.2):
  * a projection jump rejects THIS cycle: accepted-arc baseline frozen, and
    the reject-phase command is speed-limited (A5.0 item 3);
  * a stationary robot banks NO budget, so a jump injected after 20 idle
    cycles is still rejected (anti-teleport);
  * a persistently rejected anchor triggers A4.2 reacquire seeking and the
    robot recovers to a new committed baseline, then normal mode;
  * the accepted baseline is forward-only (backward drift never lowers it).
"""
import math

import pytest

from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, KinematicState, MpcParams
from trajectory_tools.reference_trajectory import make_straight


def _ctrl():
    c = LinearMpcController(MpcParams())
    c.set_reference(make_straight(length=8.0))  # +x, all gear +1
    return c


def _st(x, v=0.5):
    return KinematicState(x=x, y=0.0, yaw=0.0, v=v, omega=0.0)


def test_jump_rejects_and_freezes_accepted_arc():
    ctrl = _ctrl()
    o1 = ctrl.compute_cycle(_st(1.0))           # anchor cycle
    assert o1.diag.accepted_arc == pytest.approx(1.0, abs=1e-6)
    o2 = ctrl.compute_cycle(_st(1.0))           # no motion -> accept, no bank
    assert o2.diag.health == HealthState.OK
    assert ctrl._budget == 0.0
    o3 = ctrl.compute_cycle(_st(4.0))           # 3 m pose teleport
    assert o3.diag.accepted_arc == pytest.approx(1.0, abs=1e-6), "baseline moved on a rejected jump"
    assert ctrl._reject_run == 1
    assert o3.v_cmd <= MpcParams().v_probation + 1e-9, "reject phase must be speed-limited"


def test_stationary_robot_banks_nothing_jump_still_rejected():
    ctrl = _ctrl()
    ctrl.compute_cycle(_st(1.0, v=0.0))
    for _ in range(20):
        o = ctrl.compute_cycle(_st(1.0, v=0.0))  # robot standing still
        assert o.diag.health == HealthState.OK
    assert ctrl._budget == 0.0, "stationary robot accumulated allowance"
    o = ctrl.compute_cycle(_st(1.25, v=0.0))     # 0.25 m jump > step cap 0.09
    assert o.diag.accepted_arc == pytest.approx(1.0, abs=1e-6), "0.25 m teleport accepted after 20 idle cycles"
    assert ctrl._reject_run == 1


def test_reacquire_recovers_committed_baseline_then_normal():
    ctrl = _ctrl()
    ctrl.compute_cycle(_st(1.0))
    ctrl.compute_cycle(_st(4.0))                 # reject #1
    saw_seeking = False
    for _ in range(60):                          # robot parked at 4.0
        o = ctrl.compute_cycle(_st(4.0))
        if o.diag.reason == "REACQUIRE_SEEKING":
            saw_seeking = True
    assert saw_seeking, "persistent rejection must enter A4.2 seeking"
    assert o.diag.accepted_arc == pytest.approx(4.0, abs=1e-6), "reacquire did not commit the true baseline"
    assert o.diag.in_probation is False, "probation should have expired"
    assert ctrl._reacquire_events >= 1
    # and the robot can keep tracking forward afterwards
    o2 = ctrl.compute_cycle(_st(4.04))
    assert o2.diag.health == HealthState.OK
    assert o2.diag.accepted_arc == pytest.approx(4.04, abs=1e-6)


def test_baseline_is_forward_only_within_back_m():
    ctrl = _ctrl()
    ctrl.compute_cycle(_st(1.0))
    for x in (1.05, 1.10, 1.15, 1.20, 1.25):    # honest forward advance
        o = ctrl.compute_cycle(_st(x))
        assert o.diag.health == HealthState.OK
    assert ctrl._baseline == pytest.approx(1.25, abs=1e-6)
    o = ctrl.compute_cycle(_st(1.20))            # small backward drift < back_m
    assert o.diag.health == HealthState.OK
    assert o.diag.accepted_arc == pytest.approx(1.25, abs=1e-6), "baseline moved backwards"


def test_honest_forward_never_rejected_and_no_reacquire():
    ctrl = _ctrl()
    ctrl.compute_cycle(_st(0.0))
    reacquired = False
    for k in range(1, 120):
        o = ctrl.compute_cycle(_st(0.04 * k))
        assert o.diag.health == HealthState.OK
        reacquired = reacquired or o.diag.reason == "REACQUIRE_SEEKING"
    assert not reacquired
    assert ctrl._baseline == pytest.approx(0.04 * 119, abs=2e-6)
