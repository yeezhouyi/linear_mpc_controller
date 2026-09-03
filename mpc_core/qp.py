"""Small self-contained dense QP solver (ADMM, OSQP-style) + MPC QP builder.

Realtime product code uses OSQP through the C++ core (WSL2).  This numpy
implementation is the *offline reference*: it solves

    min  0.5 x' P x + q' x
    s.t. l <= A x <= u           (entries may be +/-inf -> one-sided rows)

with a scaled ADMM (Boyd et al. 2011, "Distributed optimization and
statistical learning via ADMM", Sec. 5.2 / OSQP paper).  It exposes the same
conceptual contract as the C++ ``OSQPSolver``: status, iterations, residuals,
objective, warm start, so unit tests can run on either backend.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

INF = float("inf")


@dataclass
class QpResult:
    status: str = "FAILED"          # SOLVED | APPROXIMATE | FAILED
    x: Optional[np.ndarray] = None
    iterations: int = 0
    pri_res: float = INF
    dua_res: float = INF
    objective: float = 0.0
    solve_time_us: int = 0

    @property
    def ok(self) -> bool:
        return self.status in ("SOLVED", "APPROXIMATE") and self.x is not None and bool(
            np.all(np.isfinite(self.x))
        )


def _clip(x: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)


class AdmmQp:
    """Dense scaled-ADMM QP solver (projection onto the box [l, u])."""

    def __init__(
        self,
        max_iter: int = 1500,
        abs_tol: float = 1e-6,
        rel_tol: float = 1e-5,
        rho: float = 1.0,
    ) -> None:
        self.max_iter = int(max_iter)
        self.abs_tol = float(abs_tol)
        self.rel_tol = float(rel_tol)
        self.rho = float(rho)

    def solve(
        self,
        P: np.ndarray,
        q: np.ndarray,
        A: np.ndarray,
        l: np.ndarray,
        u: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
    ) -> QpResult:
        n = P.shape[0]
        m = A.shape[0]
        t0 = time.perf_counter()

        # sanity
        if not np.all(np.isfinite(P)) or not np.all(np.isfinite(q)):
            return QpResult(status="FAILED")
        l = np.asarray(l, dtype=float)
        u = np.asarray(u, dtype=float)

        # Slight regularisation: P is PD for our MPC cost, but keep safe.
        P_sym = 0.5 * (P + P.T)
        rho = self.rho
        At = A.T
        AtA = At @ A
        # M = P + rho A'A is constant while rho is fixed: factor once with
        # Cholesky and solve by triangular back-substitution each iteration.
        M = P_sym + rho * AtA
        try:
            L = np.linalg.cholesky(M)
        except np.linalg.LinAlgError:
            return QpResult(status="FAILED")
        Lh = L.T

        x = np.zeros(n)
        if warm_start is not None and warm_start.size == n and np.all(np.isfinite(warm_start)):
            x = np.asarray(warm_start, dtype=float).copy()
        Ax = A @ x
        z = _clip(Ax, l, u)
        y = np.zeros(m)

        eps_abs = self.abs_tol
        eps_rel = self.rel_tol
        status = "FAILED"
        pri_res = dua_res = INF
        x_sol = x

        for it in range(1, self.max_iter + 1):
            # ---- x update (pre-factored linear system) ----
            rhs = At @ (rho * z - y) - q
            t = np.linalg.solve(L, rhs)
            x = np.linalg.solve(Lh, t)
            if not np.all(np.isfinite(x)):
                return QpResult(status="FAILED", iterations=it)

            # ---- z update (projection) ----
            Ax = A @ x
            z_new = _clip(Ax + y / rho, l, u)

            # ---- y update ----
            y = y + rho * (Ax - z_new)

            # ---- residuals ----
            if m > 0:
                pri_res = float(np.max(np.abs(Ax - z_new)))
                dua_res = float(rho * np.max(np.abs(At @ (z_new - z))))
                eps_p = eps_abs + eps_rel * max(float(np.max(np.abs(Ax))), float(np.max(np.abs(z))))
                eps_d = eps_abs + eps_rel * float(np.max(np.abs(At @ y)))
            else:
                pri_res = dua_res = 0.0
                eps_p = eps_d = 0.0
            z = z_new
            x_sol = x

            if pri_res <= eps_p and dua_res <= eps_d:
                status = "SOLVED"
                break

        if status != "SOLVED":
            # Accept only clearly-converged-enough iterates (OSQP's
            # "solved inaccurate" analogue): residuals within 10x tolerance.
            if m == 0:
                status = "SOLVED"
            elif pri_res <= 10.0 * (eps_abs + eps_rel * max(np.max(np.abs(Ax)), np.max(np.abs(z)))) and \
                    dua_res <= 10.0 * (eps_abs + eps_rel * max(float(np.max(np.abs(At @ y))), 1e-12)):
                status = "APPROXIMATE"
            else:
                status = "FAILED"

        obj = 0.5 * float(x_sol @ (P_sym @ x_sol)) + float(q @ x_sol)
        t_us = int((time.perf_counter() - t0) * 1e6)
        return QpResult(
            status=status,
            x=x_sol.copy(),
            iterations=int(it) if status != "FAILED" else self.max_iter,
            pri_res=pri_res,
            dua_res=dua_res,
            objective=obj,
            solve_time_us=t_us,
        )
