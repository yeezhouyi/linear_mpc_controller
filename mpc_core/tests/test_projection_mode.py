"""A8 instrumentation tests: controller_projection_mode + A7.2 item 6.

* windowed (default, A5 gated) vs global (raw-arc, pre-A5): over a pose
  sequence that contains a 3 m projection jump, the gated controller's
  accepted progress is STRICTLY LESS than the raw controller's -- the
  windowed projection must never silently grant a jump as progress
  (A7.2 item 6, exercised on the canonical jump; the corner-cut variant
  needs the u9 fold geometry and is tracked with item 3).
"""
import pytest

from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, KinematicState, MpcParams
from trajectory_tools.reference_trajectory import make_straight


def _st(x, v=0.5):
    return KinematicState(x=x, y=0.0, yaw=0.0, v=v, omega=0.0)


def test_global_mode_accepts_every_raw_projection():
    ctrl = LinearMpcController(MpcParams(controller_projection_mode="global"))
    ctrl.set_reference(make_straight(length=8.0))
    o1 = ctrl.compute_cycle(_st(1.0))
    assert o1.diag.accepted_arc == pytest.approx(1.0, abs=1e-6)
    o2 = ctrl.compute_cycle(_st(4.0))     # 3 m teleport
    assert o2.diag.accepted_arc == pytest.approx(4.0, abs=1e-6), \
        "global mode must follow the raw arc (no gate)"


def test_windowed_grants_strictly_less_progress_than_global():
    def accepted_after(params, poses):
        ctrl = LinearMpcController(MpcParams(**params))
        ctrl.set_reference(make_straight(length=8.0))
        last = 0.0
        for x in poses:
            out = ctrl.compute_cycle(_st(x))
            last = out.diag.accepted_arc
            if ctrl._reacquire_mode != "NORMAL" or out.diag.reason:
                break
        return last

    poses = [1.0] * 10 + [4.0] + [4.04] * 20   # idle, then a 3 m jump
    g = accepted_after({"controller_projection_mode": "global"}, poses)
    w = accepted_after({"controller_projection_mode": "windowed"}, poses)
    assert g == pytest.approx(4.04, abs=1e-6)   # raw controller books the jump and keeps going
    assert w < g - 0.5, \
        f"windowed granted {w} >= global {g}: the gate rubber-stamped a jump"


def test_windowed_default_preserves_gated_behaviour():
    ctrl = LinearMpcController(MpcParams())     # default = windowed
    assert ctrl.params.controller_projection_mode == "windowed"
    ctrl.set_reference(make_straight(length=8.0))
    ctrl.compute_cycle(_st(1.0))
    o = ctrl.compute_cycle(_st(4.0))
    assert o.diag.accepted_arc == pytest.approx(1.0, abs=1e-6), \
        "default mode must keep the acceptance gate active"
