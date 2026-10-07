"""Focused T04 contracts for the isolated Python repair tree."""

import inspect
import typing

import numpy as np

from mpc_core import mpc as mpc_module
from mpc_core.mpc import LinearMpcController
from mpc_core.qp import AdmmQp, QpResult
from mpc_core.types import MpcParams
from mpc_core.types import HealthState, KinematicState
from trajectory_tools.reference_trajectory import make_straight


def test_mpc_module_resolves_tuple_symbol_and_source_is_explicit() -> None:
    # ``from __future__ import annotations`` defers evaluation, so also check
    # the module namespace used by strict type-hint consumers.
    assert hasattr(mpc_module, "Tuple")
    assert "from typing import List, Optional, Tuple" in inspect.getsource(mpc_module)
    typing.get_type_hints(mpc_module.first_control_acceleration)


def test_reference_change_resets_reacquire_event_count() -> None:
    controller = LinearMpcController(MpcParams())
    controller.set_reference(make_straight(length=3.0))
    controller._reacquire_events = 3
    controller.set_reference(make_straight(length=3.0))
    assert controller._reacquire_events == 0


def test_constraint_diagnostic_includes_input_acceleration_bounds() -> None:
    params = MpcParams(N=2)
    controller = LinearMpcController(params)
    As = [np.eye(4), np.eye(4)]
    Bs = [np.zeros((4, 2)), np.zeros((4, 2))]
    U = np.zeros(4)
    U[0] = params.a_max + 0.25
    U[3] = -(params.alpha_max + 0.5)
    violation = controller._max_violation(
        np.zeros(4), As, Bs, [None, None, None], U
    )
    assert violation >= 0.5


def test_explicit_admm_deadline_and_controller_mapping() -> None:
    result = AdmmQp(max_iter=100_000, time_limit_s=1.0e-12).solve(
        np.eye(2), np.zeros(2), np.eye(2), -np.ones(2), np.ones(2)
    )
    assert result.status == "TIMEOUT"

    class TimeoutSolver:
        def solve(self, *args, **kwargs):
            return QpResult(status="TIMEOUT", iterations=1)

    controller = LinearMpcController(
        MpcParams(), make_straight(length=3.0), solver=TimeoutSolver()
    )
    output = controller.compute_cycle(
        KinematicState(x=0.0, y=0.0, yaw=0.0, v=0.15, omega=0.0)
    )
    assert output.diag.health == HealthState.QP_TIMEOUT
    assert output.diag.qp_status == "TIMEOUT"
    assert output.v_cmd == 0.0
    assert output.omega_cmd == 0.0
