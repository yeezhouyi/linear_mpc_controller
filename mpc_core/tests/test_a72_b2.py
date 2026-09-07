"""A7.2 behaviour tests, batch 2 (items 2 / 7 / 11 / 12 of the merged list).

  * item 2  -- a new reference resets the controller projection state
  * item 7  -- genuinely equidistant lanes with no s_prev -> stage 3 and the
               upper layer stops (no silent coin-flip argmin)
  * item 11 -- cusp segment_gear: a +1 -> 0 -> -1 reference selects the
               correct side before and after the cusp; the reverse side is
               NOT rejected by the heading gate (A2.2)
  * item 12 -- stage 3 never reaches the QP solver (mock-counted)
"""
import math

import numpy as np
import pytest

from mpc_core.frenet import closest_point
from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, KinematicState, MpcParams, Trajectory
from trajectory_tools.reference_trajectory import make_straight


# ------------------------------------------------------------- item 2
def test_new_reference_resets_projection_state():
    ctrl = LinearMpcController(MpcParams())
    ctrl.set_reference(make_straight(length=8.0))
    ctrl.compute_cycle(KinematicState(x=1.0, y=0.0, yaw=0.0, v=0.5, omega=0.0))
    assert ctrl._baseline is not None
    ctrl.set_reference(make_straight(length=8.0))   # new Path
    assert ctrl._baseline is None
    assert ctrl._last_xy is None
    assert ctrl._budget == 0.0
    assert ctrl._reject_run == 0
    o = ctrl.compute_cycle(KinematicState(x=1.0, y=0.0, yaw=0.0, v=0.5, omega=0.0))
    assert o.diag.health == HealthState.OK
    assert o.diag.accepted_arc == pytest.approx(1.0, abs=1e-6)


# ------------------------------------------------------------- item 7
def _antiparallel_traj(length=2.0, offset=0.30, n=101):
    """Out along +x at y=0, back along -x at y=offset (arc keeps rising)."""
    fx = np.linspace(0.0, length, n)
    rx = np.linspace(length, 0.0, n)
    x = np.concatenate([fx, rx])
    y = np.concatenate([np.zeros(n), np.full(n, offset)])
    yaw = np.concatenate([np.zeros(n), np.full(n, math.pi)])
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=np.zeros(x.size),
                      v=np.full(x.size, 0.5))


def test_equidistant_lanes_without_s_prev_is_ambiguous_stage3():
    """Robot exactly between two antiparallel lanes (0.15 m from each): no
    s_prev exists to break the tie -> stage 3, never an arbitrary pick."""
    traj = _antiparallel_traj()
    seg, w, e_y, arc, stage = closest_point(traj, 1.0, 0.15)
    assert stage == 3
    assert seg == -1
    assert math.isnan(e_y)
    # a heading perpendicular to BOTH lanes also leaves nothing trustworthy
    # (heading gate rejects every segment -> stage 3, binding, no fallback)
    seg2, _, e2, arc2, stage2 = closest_point(traj, 1.0, 0.15,
                                              yaw=math.pi / 2.0)
    assert stage2 == 3
    assert math.isnan(e2)


def test_with_s_prev_equidistant_lanes_resolve_toward_s_prev():
    traj = _antiparallel_traj()
    seg, w, e_y, arc, stage = closest_point(traj, 1.0, 0.15, s_prev=1.0)
    assert stage < 3
    assert arc == pytest.approx(1.0, abs=1e-6)


# ------------------------------------------------------------- item 11
def test_cusp_segment_gear_selects_correct_sides():
    """Forward (+1) -> zero-speed cusp -> reverse (-1); poses on each side
    select their own segment and the reverse side passes the heading gate
    (direction comes from the explicit gear, not from sign(v)=0 at the cusp).
    """
    n = 101
    fx = np.linspace(0.0, 2.0, n)
    rx = np.linspace(2.0, 0.0, n)
    x = np.concatenate([fx, rx])        # 2n points: fx ends 2.0, rx starts 2.0
    y = np.zeros(x.size)
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + abs(x[i] - x[i - 1])
    v = np.zeros(x.size)
    v[: n - 1] = 0.8                    # forward motion points
    v[n - 1] = 0.0                      # zero-speed cusp point
    v[n:] = -0.5                        # reverse motion points
    # per-segment gear (N-1 = 2n-1): forward segs +1; the degenerate cusp
    # segment and the reverse run belong to the REVERSE motion (-1).
    gear = np.concatenate([np.ones(n - 1), np.full(n, -1.0)])
    traj = Trajectory(s=s, x=x, y=y, yaw=np.zeros(x.size),
                      kappa=np.zeros(x.size), v=v, segment_gear=gear)
    cusp_s = float(traj.s[n])             # arc at the cusp (x = 2)
    seg_f, _, e_f, arc_f, st_f = closest_point(
        traj, 1.95, 0.0, yaw=0.0, s_prev=cusp_s - 0.5)
    assert st_f < 3, "forward side rejected by the heading gate"
    assert arc_f < cusp_s, f"forward pose picked the reverse side (arc {arc_f})"
    # just AFTER the cusp, driving REVERSE: the body faces +x (yaw 0) while
    # travel is -x; the explicit gear -1 must keep the gate open.
    seg_r, _, e_r, arc_r, st_r = closest_point(
        traj, 1.95, 0.0, yaw=0.0, s_prev=cusp_s + 0.5)
    assert st_r < 3, "reverse side rejected by the heading gate (gear ignored?)"
    assert arc_r > cusp_s, f"reverse pose picked the forward side (arc {arc_r})"
    # declaring a uniform FORWARD gear must leave the heading gate rejecting
    # the reverse lane at this pose (motion +x vs reverse tangent -x).
    seg_w, _, _, arc_w, st_w = closest_point(
        traj, 1.95, 0.0, yaw=0.0, s_prev=cusp_s + 0.5, travel_sign=+1)
    if st_w < 3:
        assert arc_w < cusp_s, "forward gear accepted the reverse lane"


# ------------------------------------------------------------- item 12
def test_stage3_never_reaches_qp_solver():
    """A3.2: on stage == 3 the controller must stop BEFORE building any QP
    input -- mock the solver and assert it is never invoked."""
    ctrl = LinearMpcController(MpcParams())
    ctrl.set_reference(make_straight(length=8.0))
    calls = []
    orig_solve = ctrl.solver.solve
    ctrl.solver.solve = lambda *a, **k: calls.append(1) or orig_solve(*a, **k)
    st = KinematicState(x=1.0, y=0.0, yaw=math.pi, v=0.0, omega=0.0)
    out = ctrl.compute_cycle(st)
    assert calls == [], f"solve_qp was invoked {len(calls)} times on stage 3"
    assert out.diag.health == HealthState.EMERGENCY_STOP
    assert out.diag.reason == "PROJECTION_AMBIGUOUS"
    assert out.diag.qp_status == ""
