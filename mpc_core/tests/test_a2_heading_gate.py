"""A2 heading-gate behaviour tests (fail-open closed, reverse gear support).

Corresponds to the 4060 doc A2/A2.2 acceptance items that are landable with
the geometry layer (the A5.1 acceptance-gate wiring unit is separate):
  * heading gate rejecting EVERY segment returns stage == 3 -- there is NO
    unconstrained global-argmin fallback (fail-open defect closed);
  * reverse travel is supported via the EXPLICIT per-segment gear (or the
    test-only uniform travel_sign override), never inferred from velocity
    sign -- a reverse-driven segment is not rejected by the heading gate;
  * the stateless path stays deterministic (recorded deviation: no-s_prev
    near-tie resolves to the old argmin, stage 2);
  * LinearMpcController stops BEFORE building QP input on stage == 3
    (A3.2): zero command, EMERGENCY_STOP/PROJECTION_AMBIGUOUS, QP never run.
"""
import math

import numpy as np
import pytest

from mpc_core.frenet import closest_point
from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, KinematicState, MpcParams, Trajectory
from trajectory_tools.reference_trajectory import make_straight


def _straight():
    return make_straight(length=8.0)  # +x, gear default all +1


def test_heading_gate_rejects_all_returns_stage3():
    """A robot facing pi on an all-forward path is on NO lane: stage 3, not
    an unconstrained argmin.  Fail-open closed."""
    traj = _straight()
    seg, w, e_y, arc, stage = closest_point(
        traj, 1.0, 0.0, yaw=math.pi, s_prev=0.5)
    assert stage == 3
    assert seg == -1
    assert math.isnan(e_y)
    assert arc == pytest.approx(0.5)


def test_reverse_gear_allows_reverse_heading():
    """Same pose/heading but declared reverse travel: the heading gate must
    accept it (A2.2 -- direction from the explicit gear, not from v)."""
    traj = _straight()
    seg, w, e_y, arc, stage = closest_point(
        traj, 1.0, 0.0, yaw=math.pi, s_prev=0.98, travel_sign=-1)
    assert stage < 3  # accepted (window stage 0/1/2, never 3)
    assert seg >= 0
    assert e_y == pytest.approx(0.0, abs=1e-9)
    assert arc == pytest.approx(1.0, abs=0.03)  # window anchor, within one micro-segment


def test_forward_heading_is_accepted_with_window():
    traj = _straight()
    seg, w, e_y, arc, stage = closest_point(
        traj, 1.0, 0.05, yaw=0.0, s_prev=0.98)
    assert stage == 0  # narrow window contains the true foot
    assert e_y == pytest.approx(0.05, abs=1e-9)
    assert arc == pytest.approx(1.0, abs=0.03)


def test_stateless_on_vertex_pose_stays_deterministic():
    """Recorded deviation: pose exactly over a sampled vertex (equidistant to
    two adjacent micro-segments) with no s_prev resolves to the old argmin
    (stage 2), not stage 3 -- keeps the stateless search deterministic."""
    traj = _straight()
    r1 = closest_point(traj, 2.0, 0.4)
    r2 = closest_point(traj, 2.0, 0.4)
    assert r1 == r2
    assert r1[4] == 2
    assert r1[2] == pytest.approx(0.4, abs=1e-9)


def test_controller_stops_pre_qp_on_stage3():
    """Robot facing pi on a forward path at the first cycle: the controller
    must refuse BEFORE building any QP input (A3.2) -- zero command, no QP
    status, EMERGENCY_STOP/PROJECTION_AMBIGUOUS."""
    ctrl = LinearMpcController(MpcParams())
    ctrl.set_reference(_straight())
    st = KinematicState(x=1.0, y=0.0, yaw=math.pi, v=0.0, omega=0.0)
    out = ctrl.compute_cycle(st)
    assert out.v_cmd == 0.0 and out.omega_cmd == 0.0
    assert out.diag.health == HealthState.EMERGENCY_STOP
    assert out.diag.reason == "PROJECTION_AMBIGUOUS"
    assert out.diag.fallback_stage == 3
    assert out.diag.qp_status == ""  # QP was never invoked


def test_controller_recovers_after_alignment():
    """Stage-3 stop is not a latch: once the heading is path-aligned the
    next cycle tracks normally (s_prev frozen on the refused cycle only)."""
    traj = _straight()
    ctrl = LinearMpcController(MpcParams())
    ctrl.set_reference(traj)
    st = KinematicState(x=1.0, y=0.0, yaw=math.pi, v=0.0, omega=0.0)
    out1 = ctrl.compute_cycle(st)
    assert out1.diag.reason == "PROJECTION_AMBIGUOUS"
    st2 = KinematicState(x=1.0, y=0.0, yaw=0.0, v=0.0, omega=0.0)
    out2 = ctrl.compute_cycle(st2)
    assert out2.diag.reason != "PROJECTION_AMBIGUOUS"
    assert out2.diag.health == HealthState.OK
