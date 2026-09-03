"""Differential-drive kinematics, error dynamics, discretization (U2).

The authoritative derivation is docs/mpc_model_derivation.md.  This module
implements

1. the *nonlinear* Frenet error dynamics used for validation/rollout,
2. the analytic linearisation (LTV) ``A_c, B_c`` at the local reference
   ``(v_r, kappa)``,
3. zero-order-hold / Euler discretisation, and
4. a small unicycle plant (perfect or first-order lag) used by the fast RL
   environment and the offline demos.

State/input freeze (R1/R2):
    x = [e_y, e_psi, v, omega],   u = [a, alpha]  (continuous accel inputs)
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from mpc_core.types import (
    A_IDX,
    ALPHA_IDX,
    EY_IDX,
    EPSI_IDX,
    NX,
    NU,
    OMEGA_IDX,
    V_IDX,
    KinematicState,
    TrackPoint,
    Trajectory,
    wrap_angle,
)

INF = float("inf")


# --------------------------------------------------------------------------
# Nonlinear continuous dynamics (validation / truth model in Frenet frame)
# --------------------------------------------------------------------------

def nonlinear_error_derivative(
    x: np.ndarray, v_r: float, kappa: float
) -> np.ndarray:
    """Continuous derivative of the Frenet error state (exact kinematics).

    ``x = [e_y, e_psi, v, omega]``; ``kappa`` is the *frozen* path curvature
    at the current anchor (piecewise-constant over a step).

    Conventions: ``e_y > 0`` = robot LEFT of travel (for a CCW turn with
    ``kappa > 0`` the centre of curvature is on the left, so ``e_y`` points
    toward the centre); ``e_psi = wrap(psi - psi_ref)``.
    """
    e_y, e_psi, v, omega = x
    denom = 1.0 - kappa * e_y
    if abs(denom) < 1e-9:
        # Never divide by (almost) zero: robot on the centre of curvature.
        denom = np.sign(denom) * 1e-9 if denom != 0 else 1e-9
    d_e_y = v * np.sin(e_psi)
    d_e_psi = omega - kappa * v * np.cos(e_psi) / denom
    return np.array([d_e_y, d_e_psi, 0.0, 0.0], dtype=float)


def linear_continuous_matrices(
    v_r: float, kappa: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Analytic LTV matrices linearised about (0, 0, v_r, kappa*v_r)."""
    A = np.zeros((NX, NX))
    A[EY_IDX, EPSI_IDX] = v_r
    A[EPSI_IDX, EY_IDX] = -kappa * kappa * v_r
    A[EPSI_IDX, V_IDX] = -kappa
    A[EPSI_IDX, OMEGA_IDX] = 1.0
    B = np.zeros((NX, NU))
    B[V_IDX, A_IDX] = 1.0
    B[OMEGA_IDX, ALPHA_IDX] = 1.0
    return A, B


# --------------------------------------------------------------------------
# Matrix exponential (scaling & squaring + Taylor) -- numpy-only
# --------------------------------------------------------------------------

def expm_dense(M: np.ndarray, degree: int = 16) -> np.ndarray:
    """Dense matrix exponential (scaling & squaring with Taylor series).

    Deterministic and dependency-free.  Used for exact ZOH discretisation.
    """
    M = np.asarray(M, dtype=float)
    n = M.shape[0]
    norm = float(np.linalg.norm(M, ord=np.inf))
    s = max(0, int(np.ceil(np.log2(norm))) ) if norm > 1.0 else 0
    A = M / (2.0 ** s)
    # Taylor series
    acc = np.eye(n)
    term = np.eye(n)
    for k in range(1, degree + 1):
        term = term @ A / k
        acc = acc + term
    for _ in range(s):
        acc = acc @ acc
    return acc


def discretize_zoh(A: np.ndarray, B: np.ndarray, Ts: float) -> Tuple[np.ndarray, np.ndarray]:
    """Zero-order-hold discretisation via the augmented matrix exponential.

    ``x[k+1] = A_d x[k] + B_d u[k]`` with u held constant over ``Ts``.
    """
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm_dense(M * Ts)
    return E[:n, :n], E[:n, n:]


def discretize_euler(A: np.ndarray, B: np.ndarray, Ts: float) -> Tuple[np.ndarray, np.ndarray]:
    """First-order forward-Euler discretisation (kept for error comparison)."""
    return np.eye(A.shape[0]) + Ts * A, Ts * B


def discretize_step(A_c, B_c, Ts: float, method: str = "zoh") -> Tuple[np.ndarray, np.ndarray]:
    if method == "zoh":
        return discretize_zoh(A_c, B_c, Ts)
    if method == "euler":
        return discretize_euler(A_c, B_c, Ts)
    raise ValueError(f"unknown discretization method {method}")


# --------------------------------------------------------------------------
# LTV window over the prediction horizon
# --------------------------------------------------------------------------

def build_ltv_window(
    traj: Trajectory,
    base_arc: float,
    Ts: float,
    N: int,
    method: str = "zoh",
) -> Tuple[List[np.ndarray], List[np.ndarray], List[TrackPoint]]:
    """Sample the reference every Ts assuming the robot advances at v_ref.

    Returns parallel lists (length N) of ``A_d[j], B_d[j]`` plus the ``N+1``
    reference anchors used to build the horizon reference state.
    """
    # preview arcs: advance at reference speed
    arcs: List[float] = []
    s = base_arc
    for _ in range(N + 1):
        arcs.append(s)
        if s >= traj.s[-1]:
            break
        pt = traj.sample_by_s(s)
        s = s + Ts * max(pt.v, 0.0)
    # pad the last arc at the end
    while len(arcs) < N + 1:
        arcs.append(traj.s[-1])

    As, Bs, anchors = [], [], []
    for j in range(N):
        pt = traj.sample_by_s(arcs[j])
        A_c, B_c = linear_continuous_matrices(max(pt.v, 1e-9), pt.kappa)
        A_d, B_d = discretize_step(A_c, B_c, Ts, method)
        As.append(A_d)
        Bs.append(B_d)
    anchors = [traj.sample_by_s(a) for a in arcs[: N + 1]]
    return As, Bs, anchors


def reference_state_vector(anchor: TrackPoint) -> np.ndarray:
    """Reference in error-state coordinates: [0, 0, v_r, kappa*v_r]."""
    return np.array([0.0, 0.0, anchor.v, anchor.kappa * anchor.v], dtype=float)


# --------------------------------------------------------------------------
# Unicycle plant (fast env / demos)
# --------------------------------------------------------------------------

class DifferentialDrivePlant:
    """2-D pose integrator receiving (v_cmd, omega_cmd).

    ``lag_s`` == 0 -> perfect velocity tracking (actual velocity == command),
    otherwise a first-order lag ``d(v)/dt = (v_cmd - v)/lag_s`` (the mismatch
    scenario the residual-RL stage is designed to compensate, see KTD8/R10).
    """

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        yaw0: float = 0.0,
        v0: float = 0.0,
        omega0: float = 0.0,
        lag_s: float = 0.0,
        seed: Optional[int] = None,
    ) -> None:
        self.state = KinematicState(x=x0, y=y0, yaw=yaw0, v=v0, omega=omega0)
        self.lag_s = float(lag_s)
        self.rng = np.random.default_rng(seed)

    def reset(self, x0=0.0, y0=0.0, yaw0=0.0, v0=0.0, omega0=0.0) -> KinematicState:
        self.state = KinematicState(x=x0, y=y0, yaw=yaw0, v=v0, omega=omega0)
        return self.state

    def step(self, v_cmd: float, omega_cmd: float, Ts: float) -> KinematicState:
        st = self.state
        if self.lag_s > 1e-12:
            tau = self.lag_s
            v = st.v + (Ts / tau) * (v_cmd - st.v)
            omega = st.omega + (Ts / tau) * (omega_cmd - st.omega)
        else:
            v, omega = float(v_cmd), float(omega_cmd)
        yaw = wrap_angle(st.yaw + omega * Ts)
        x = st.x + v * np.cos(yaw) * Ts
        y = st.y + v * np.sin(yaw) * Ts
        self.state = KinematicState(x=float(x), y=float(y), yaw=yaw, v=v, omega=omega)
        return self.state


def integrate_state(x0: np.ndarray, u_seq: np.ndarray, As, Bs) -> np.ndarray:
    """Roll the discrete LTV model forward; returns states (N+1, 4)."""
    xs = np.zeros((len(As) + 1, NX))
    xs[0] = x0
    for j, (A, B) in enumerate(zip(As, Bs)):
        xs[j + 1] = A @ xs[j] + B @ u_seq[j]
    return xs
