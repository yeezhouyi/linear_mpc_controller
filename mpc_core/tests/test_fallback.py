"""Fallback & health contract tests (U4)."""
import math

import pytest

from mpc_core.fallback import FallbackPolicy
from mpc_core.types import HealthState, MpcDiagnostics, MpcParams


def mk_diag():
    return MpcDiagnostics()


def test_healthy_output_passes_and_clamps():
    p = MpcParams(v_min=0.0, v_max=1.5, omega_max=2.0)
    fb = FallbackPolicy(p)
    diag = mk_diag()
    v, w = fb.apply((1.0, 0.5), HealthState.OK, diag)
    assert (v, w) == (1.0, 0.5)
    assert diag.health == HealthState.OK
    assert not diag.clamped

    diag = mk_diag()
    v, w = fb.apply((2.0, 3.0), HealthState.OK, diag)
    assert (v, w) == (1.5, 2.0)  # clamped to hard bounds
    assert diag.health == HealthState.SAFETY_CLAMPED
    assert diag.clamped


def test_nan_never_published():
    fb = FallbackPolicy(MpcParams())
    diag = mk_diag()
    v, w = fb.apply((float("nan"), 1.0), HealthState.OK, diag)
    assert (v, w) == (0.0, 0.0)
    assert diag.health == HealthState.NAN_OUTPUT
    assert diag.fallback_stage == 3


def test_critical_health_immediate_zero():
    fb = FallbackPolicy(MpcParams())
    for hs in (HealthState.NO_REFERENCE, HealthState.EMERGENCY_STOP, HealthState.STATE_STALE):
        diag = mk_diag()
        v, w = fb.apply((0.8, 0.3), hs, diag)
        assert (v, w) == (0.0, 0.0)
        assert diag.fallback_stage == 3


def test_qp_failure_degrades_then_emergency_stops():
    p = MpcParams(Ts=0.05, stop_deceleration=0.8, max_hold_cycles=6)
    fb = FallbackPolicy(p)
    # simulate repeated QP failures starting from v=1.0
    v0 = 1.0
    speeds = []
    for _ in range(10):
        diag = mk_diag()
        v, w = fb.apply((v0, 0.0), HealthState.QP_INFEASIBLE, diag)
        speeds.append(v)
        if diag.health == HealthState.EMERGENCY_STOP:
            assert v == 0.0
            break
    # strictly decreasing to zero and emergency stop hit within budget
    assert speeds[0] < v0 or speeds[0] == pytest.approx(v0)
    assert all(s2 <= s1 + 1e-12 for s1, s2 in zip(speeds, speeds[1:]))
    assert speeds[-1] == 0.0
    assert len(speeds) <= p.max_hold_cycles + 1


def test_degrade_never_negative():
    fb = FallbackPolicy(MpcParams(Ts=0.05, stop_deceleration=1.0, max_hold_cycles=3))
    diag = mk_diag()
    v, _ = fb.apply((0.0, 0.0), HealthState.QP_TIMEOUT, diag)
    assert v >= 0.0
