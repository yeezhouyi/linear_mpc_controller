"""OSQP backend for the condensed MPC QP (task D).

Why this exists
---------------
``AdmmQp`` is the historical offline reference.  On the published
``test_straight_noise_delay`` episode it returns ``FAILED`` on six cycles after
burning all 1500 iterations, while OSQP solves the identical condensed problem
in 75--150 iterations with zero constraint violation.  The instances are
feasible and well conditioned (condition number ~7.2); the ADMM simply needs
1500--2100 iterations at its fixed ``rho``.  Those ``FAILED`` rows were
therefore an iteration-budget artefact, not infeasibility.

This module is the forward backend.  It is deliberately kept separate from the
C++ runtime's OSQP 0.6.x dependency: nothing here upgrades or links against the
ROS side.  The reported status is a *contract*, not a solver string:

``SOLVED``
    OSQP reported convergence. Inaccurate or iteration-limited results are
    never promoted to this status.
``APPROXIMATE``
    A usable iterate that is not fully converged.  ``max_violation`` always
    carries the *measured* violation of the returned plan, so callers can apply
    their own feasibility review instead of trusting a status name.
``FAILED``
    No usable plan.  ``dual_certificate`` is populated only when OSQP actually
    proved primal infeasibility; it stays ``None`` otherwise, so "we do not
    know" is never dressed up as "we proved it impossible".
"""
from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

from mpc_core.qp import INF, QpResult

try:  # pragma: no cover - exercised through the import guard test
    import osqp
    import scipy.sparse as sp

    OSQP_IMPORT_ERROR: Optional[str] = None
except ImportError as exc:  # pragma: no cover
    osqp = None  # type: ignore[assignment]
    sp = None  # type: ignore[assignment]
    OSQP_IMPORT_ERROR = str(exc)


# OSQP 1.x renamed the residual fields; accept either spelling.
_PRIM_RES = ("prim_res", "pri_res")
_DUAL_RES = ("dual_res", "dua_res")

# Statuses OSQP can return once it stops iterating without a certificate.
_ITERATION_LIMITED = {"maximum iterations reached", "time limit reached",
                      "iteration_limit_reached", "time_limit_reached"}
_TIME_LIMITED = {"time limit reached", "time_limit_reached"}
_PRIMAL_INFEASIBLE = {"primal infeasible", "primal infeasible inaccurate",
                      "primal_infeasible"}


def _info_value(info: Any, names: tuple[str, ...], default: float) -> float:
    for name in names:
        if hasattr(info, name):
            value = getattr(info, name)
            if value is not None:
                return float(value)
    return float(default)


class OsqpQp:
    """Dense QP backend with :class:`~mpc_core.qp.AdmmQp`'s call signature."""

    backend_id = "osqp-python"

    def __init__(
        self,
        max_iter: int = 1500,
        abs_tol: float = 1e-6,
        rel_tol: float = 1e-5,
        polish: bool = True,
        time_limit_s: Optional[float] = None,
        violation_tol: float = 1e-5,
        adaptive_rho: bool = True,
        scaling: int = 10,
    ) -> None:
        # Defaults mirror ``makeDefaultSolver(params_.qp_max_iter,
        # params_.qp_abs_tol, params_.qp_rel_tol)`` in the C++ runtime, which
        # also sets ``settings.polish = 1``.  An injected ``OsqpQp()`` therefore
        # runs the same solver family under the same iteration cap and
        # tolerances as ``src/mpc/osqp_solver.cpp``.
        if OSQP_IMPORT_ERROR is not None:
            raise ImportError(
                "the OSQP Python backend is required for OsqpQp: "
                f"{OSQP_IMPORT_ERROR}"
            )
        self.max_iter = int(max_iter)
        self.abs_tol = float(abs_tol)
        self.rel_tol = float(rel_tol)
        self.polish = bool(polish)
        self.time_limit_s = None if time_limit_s is None else float(time_limit_s)
        self.violation_tol = float(violation_tol)
        self.adaptive_rho = bool(adaptive_rho)
        self.scaling = int(scaling)

    def describe(self) -> dict[str, Any]:
        return {
            "backend": self.backend_id,
            "osqp_version": getattr(osqp, "__version__", "unknown"),
            "max_iter": self.max_iter,
            "abs_tol": self.abs_tol,
            "rel_tol": self.rel_tol,
            "polish": self.polish,
            "time_limit_s": self.time_limit_s,
            "violation_tol": self.violation_tol,
            "adaptive_rho": self.adaptive_rho,
            "scaling": self.scaling,
            "cpp_runtime_osqp": "0.6.x (separate; not linked here)",
        }

    def solve(
        self,
        P: np.ndarray,
        q: np.ndarray,
        A: np.ndarray,
        l: np.ndarray,
        u: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
    ) -> QpResult:
        t0 = time.perf_counter()
        n, m = P.shape[0], A.shape[0]
        P = np.asarray(P, dtype=float)
        q = np.asarray(q, dtype=float)
        if not np.all(np.isfinite(P)) or not np.all(np.isfinite(q)):
            return QpResult(status="FAILED")

        lo = np.where(np.isfinite(l), l, -np.inf)
        hi = np.where(np.isfinite(u), u, np.inf)
        # Rows unbounded on both sides carry no constraint information.
        keep = np.isfinite(lo) | np.isfinite(hi)
        A_kept, lo_kept, hi_kept = A[keep], lo[keep], hi[keep]

        problem = osqp.OSQP()
        setup_kwargs: dict[str, Any] = {
            "P": sp.csc_matrix(0.5 * (P + P.T)),
            "q": q,
            "A": sp.csc_matrix(A_kept) if m else sp.csc_matrix((0, n)),
            "l": lo_kept,
            "u": hi_kept,
            "verbose": False,
            # Python dependencies require OSQP 1.x; the C++ version is separate.
            "polishing": self.polish,
            "max_iter": self.max_iter,
            "eps_abs": self.abs_tol,
            "eps_rel": self.rel_tol,
            "adaptive_rho": self.adaptive_rho,
            "scaling": self.scaling,
        }
        if self.time_limit_s is not None:
            setup_kwargs["time_limit"] = self.time_limit_s
        try:
            problem.setup(**setup_kwargs)
            if warm_start is not None and np.size(warm_start) == n and np.all(
                np.isfinite(warm_start)
            ):
                problem.warm_start(x=np.asarray(warm_start, dtype=float))
            result = problem.solve()
        except (ValueError, TypeError) as exc:
            return QpResult(status="FAILED", solve_time_us=int(
                (time.perf_counter() - t0) * 1e6), detail=f"setup_error:{exc}")

        info = result.info
        raw_status = str(getattr(info, "status", "unsolved"))
        iterations = int(getattr(info, "iter", 0) or 0)
        objective = float(getattr(info, "obj_val", 0.0) or 0.0)
        pri_res = _info_value(info, _PRIM_RES, INF)
        dua_res = _info_value(info, _DUAL_RES, INF)
        x = None if result.x is None else np.asarray(result.x, dtype=float)

        max_violation = INF
        if x is not None and np.all(np.isfinite(x)):
            if m:
                Ax = A @ x
                max_violation = float(max(
                    0.0,
                    np.max(np.where(np.isfinite(lo), lo - Ax, -np.inf)),
                    np.max(np.where(np.isfinite(hi), Ax - hi, -np.inf)),
                ))
            else:
                max_violation = 0.0

        dual_certificate = None
        certificate = getattr(result, "prim_inf_cert", None)
        if raw_status in _PRIMAL_INFEASIBLE and certificate is not None:
            # OSQP's explicit primal-infeasibility certificate, not result.y.
            certificate = np.asarray(certificate, dtype=float)
            if np.all(np.isfinite(certificate)):
                dual_certificate = {
                    "osqp_status": raw_status,
                    "y": certificate,
                    "certificate_norm": float(np.max(np.abs(certificate))),
                }

        if x is None or not np.all(np.isfinite(x)):
            status = "FAILED"
        elif raw_status == "solved":
            status = "SOLVED"
        elif raw_status == "solved inaccurate":
            status = "APPROXIMATE" if max_violation <= self.violation_tol else "FAILED"
        elif raw_status in _TIME_LIMITED:
            status = "TIMEOUT" if max_violation <= self.violation_tol else "FAILED"
        elif raw_status in _ITERATION_LIMITED:
            status = "APPROXIMATE" if max_violation <= self.violation_tol else "FAILED"
        else:
            status = "FAILED"

        return QpResult(
            status=status,
            x=None if status == "FAILED" else x,
            iterations=iterations,
            pri_res=pri_res,
            dua_res=dua_res,
            objective=objective,
            solve_time_us=int((time.perf_counter() - t0) * 1e6),
            raw_status=raw_status,
            max_violation=max_violation,
            dual_certificate=dual_certificate,
        )
