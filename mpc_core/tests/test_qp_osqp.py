"""Task D: contract tests for the OSQP backend and its honest status reporting.

The point of these tests is not "OSQP works" but that the *reported* status
never overstates what the backend established: a run that stopped at the
iteration limit must not read as ``SOLVED``, and "no plan" must not read as
"violation 0".
"""
from __future__ import annotations

import numpy as np
import pytest

from mpc_core.qp import AdmmQp
from mpc_core.qp_osqp import OsqpQp


def _box_qp():
    """min (x0-1)^2 + (x1+2)^2 s.t. -0.5 <= x <= 0.5 (optimum: 0.5, -0.5).

    Every row is written in the one-sided form ``c'x <= u`` because that is the
    form the condensed MPC QP uses.  ``I x <= 0.5`` is the upper bound and
    ``-I x <= 0.5`` is the lower one.
    """
    P = 2.0 * np.eye(2)
    q = np.array([-2.0, 4.0])
    A = np.vstack([np.eye(2), -np.eye(2)])
    l = np.full(4, -np.inf)
    u = np.full(4, 0.5)
    return P, q, A, l, u


def test_osqp_solves_box_constrained_qp() -> None:
    P, q, A, l, u = _box_qp()
    result = OsqpQp().solve(P, q, A, l, u)
    assert result.status == "SOLVED"
    assert result.x is not None
    np.testing.assert_allclose(result.x, [0.5, -0.5], atol=1e-6)
    # The measured violation must be a real measurement, not the dataclass default.
    assert np.isfinite(result.max_violation)
    assert result.max_violation <= 1e-8


def test_unconstrained_problem_has_zero_measured_violation() -> None:
    result = OsqpQp().solve(np.eye(2), np.array([-1.0, -1.0]),
                            np.zeros((0, 2)), np.zeros(0), np.zeros(0))
    assert result.status == "SOLVED"
    # The default tolerances mirror the C++ runtime (eps_abs 1e-6), so the
    # accuracy assertion follows that contract rather than machine precision.
    np.testing.assert_allclose(result.x, [1.0, 1.0], atol=1e-5)
    assert result.max_violation == 0.0


def test_agrees_with_admm_on_a_well_behaved_problem() -> None:
    P, q, A, l, u = _box_qp()
    osqp_result = OsqpQp().solve(P, q, A, l, u)
    admm_result = AdmmQp().solve(P, q, A, l, u)
    assert osqp_result.status == "SOLVED" and admm_result.status == "SOLVED"
    np.testing.assert_allclose(osqp_result.x, admm_result.x, atol=1e-5)


def test_iteration_limit_never_reports_solved() -> None:
    """A truncated run must degrade to APPROXIMATE/FAILED, with evidence."""
    P, q, A, l, u = _box_qp()
    result = OsqpQp(max_iter=1, polish=False).solve(P, q, A, l, u)
    assert result.status in ("APPROXIMATE", "FAILED")
    assert result.raw_status != "solved"
    if result.status == "APPROXIMATE":
        assert result.x is not None
        assert result.max_violation <= OsqpQp().violation_tol


def test_infeasible_problem_is_failed_and_carries_a_certificate() -> None:
    # x <= 0 and x >= 1 cannot both hold:  x <= 0  and  -x <= -1.
    P = np.array([[1.0]])
    q = np.array([0.0])
    A = np.array([[1.0], [-1.0]])
    l = np.array([-np.inf, -np.inf])
    u = np.array([0.0, -1.0])
    result = OsqpQp().solve(P, q, A, l, u)
    assert result.status == "FAILED"
    assert result.x is None
    assert result.dual_certificate is not None, (
        "OSQP proved primal infeasibility; the certificate must be kept"
    )
    assert result.dual_certificate["certificate_norm"] > 0.0



def test_unbounded_problem_does_not_claim_primal_infeasibility() -> None:
    result = OsqpQp().solve(np.zeros((1, 1)), np.array([-1.0]),
                            np.zeros((0, 1)), np.zeros(0), np.zeros(0))
    assert result.status == "FAILED"
    assert "dual infeasible" in result.raw_status
    assert result.dual_certificate is None


def test_admm_failure_claims_no_certificate() -> None:
    """The historical backend gives up without proving anything."""
    P, q, A, l, u = _box_qp()
    result = AdmmQp(max_iter=1).solve(P, q, A, l, u)
    assert result.status == "FAILED"
    assert result.dual_certificate is None


def test_non_finite_cost_is_rejected_before_solving() -> None:
    P, q, A, l, u = _box_qp()
    bad_q = q.copy()
    bad_q[0] = np.nan
    assert OsqpQp().solve(P, bad_q, A, l, u).status == "FAILED"


def test_controller_accepts_an_injected_solver() -> None:
    from mpc_core.mpc import LinearMpcController
    from mpc_core.types import MpcParams

    params = MpcParams()
    controller = LinearMpcController(params, solver=OsqpQp())
    assert isinstance(controller.solver, OsqpQp)
    # Default stays ADMM so recorded evidence remains reproducible.
    assert isinstance(LinearMpcController(params).solver, AdmmQp)


def test_describe_reports_version_and_separates_the_cpp_runtime() -> None:
    described = OsqpQp().describe()
    assert described["backend"] == "osqp-python"
    assert described["osqp_version"] != "unknown"
    assert "separate" in described["cpp_runtime_osqp"]


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-q"]))
