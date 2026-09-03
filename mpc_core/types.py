"""Shared data types / frozen conventions for the ROS-free MPC core.

Convention freeze (see docs/mpc_model_derivation.md for the full derivation):

* World frame follows ROS: ``x`` forward, ``y`` left, ``yaw`` counter-clockwise
  positive.  A positive curvature ``kappa`` is a left turn.
* Frenet error frame is attached to the *reference* path:
    - ``e_y``  : signed lateral error, positive when the robot is LEFT of the
      travel direction of the reference tangent,
    - ``e_psi``: heading error, ``wrap(psi - psi_ref)`` in [-pi, pi),
    - ``v``    : absolute linear velocity of the robot  [m/s],
    - ``omega``: absolute angular velocity of the robot [rad/s].
* MPC state is ``x = [e_y, e_psi, v, omega]`` (4 states) and the decision
  variable is the *acceleration* pair ``u = [a, alpha] = [dv/dt, domega/dt]``
  in ``[m/s^2, rad/s^2]``.  The first predicted velocity is the ``cmd_vel``
  candidate.  Control period ``Ts`` is a model/controller parameter.
"""
from __future__ import annotations

import dataclasses
import math
from enum import IntEnum
from typing import List, Optional

import numpy as np

# State/decision dimensions (frozen, R1/R5).
NX = 4  # [e_y, e_psi, v, omega]
NU = 2  # [a, alpha]
EY_IDX, EPSI_IDX, V_IDX, OMEGA_IDX = 0, 1, 2, 3
A_IDX, ALPHA_IDX = 0, 1


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HealthState(IntEnum):
    """Structured controller health (plan U4 fallback contract)."""

    OK = 0
    NO_REFERENCE = 1
    STALE_REFERENCE = 2
    TF_INVALID = 3
    STATE_STALE = 4
    QP_TIMEOUT = 5
    QP_INFEASIBLE = 6
    NAN_OUTPUT = 7
    SAFETY_CLAMPED = 8
    FALLBACK_ACTIVE = 9
    EMERGENCY_STOP = 10

    @property
    def is_critical(self) -> bool:
        """States after which the controller must not reuse last non-zero cmd."""
        return self in (
            HealthState.NO_REFERENCE,
            HealthState.STALE_REFERENCE,
            HealthState.TF_INVALID,
            HealthState.STATE_STALE,
            HealthState.NAN_OUTPUT,
            HealthState.EMERGENCY_STOP,
        )


@dataclasses.dataclass(frozen=True)
class TrackPointKind(IntEnum):
    """Semantic kind of a reference point (used by the adapter for completion)."""

    POSE_ONLY = 0
    WITH_VELOCITY = 1
    WITH_VELOCITY_CURVATURE = 2


@dataclasses.dataclass
class TrackPoint:
    """One reference trajectory point.

    Fields:
      s     : cumulative arc length along the reference [m]
      x, y  : world position [m]
      yaw   : tangent heading of the reference at this point [rad]
      kappa : signed curvature (positive = left turn) [1/m]; 0 for straight
      v     : reference linear speed [m/s] (>= 0, forward-only for the MVP)
      t     : optional time stamp of the point (adapter domain) [s]
    """

    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    kappa: float = 0.0
    v: float = 0.0
    t: float = 0.0

    @property
    def omega(self) -> float:
        """Reference angular velocity implied by the kinematic relation."""
        return self.kappa * self.v

    def as_array(self) -> np.ndarray:
        return np.array([self.s, self.x, self.y, self.yaw, self.kappa, self.v], dtype=float)


@dataclasses.dataclass
class Trajectory:
    """A dense reference trajectory (equal-spacing optional).

    All vectors are numpy arrays of length ``n``.
    """

    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    v: np.ndarray
    kind: TrackPointKind = TrackPointKind.WITH_VELOCITY_CURVATURE

    def __post_init__(self) -> None:
        n = len(self.s)
        assert n == len(self.x) == len(self.y) == len(self.yaw) == len(self.kappa) == len(self.v)
        assert n >= 2, "trajectory needs at least two points"
        self._n = n
        self._closed = False

    @property
    def n(self) -> int:
        return self._n

    @property
    def total_length(self) -> float:
        return float(self.s[-1])

    def at_index(self, i: int) -> TrackPoint:
        i = min(max(int(i), 0), self._n - 1)
        return TrackPoint(
            s=float(self.s[i]),
            x=float(self.x[i]),
            y=float(self.y[i]),
            yaw=float(self.yaw[i]),
            kappa=float(self.kappa[i]),
            v=float(self.v[i]),
        )

    def sample_by_s(self, s: float) -> TrackPoint:
        """Linear interpolation in arc-length coordinate (clamped at ends)."""
        if s <= self.s[0]:
            return self.at_index(0)
        if s >= self.s[-1]:
            return self.at_index(self._n - 1)
        idx = int(np.searchsorted(self.s, s, side="right")) - 1
        idx = min(max(idx, 0), self._n - 2)
        s0, s1 = self.s[idx], self.s[idx + 1]
        w = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        return self._lerp(idx, idx + 1, w)

    def _lerp(self, i: int, j: int, w: float) -> TrackPoint:
        x = self.x[i] + w * (self.x[j] - self.x[i])
        y = self.y[i] + w * (self.y[j] - self.y[i])
        yaw = _lerp_angle(self.yaw[i], self.yaw[j], w)
        kappa = self.kappa[i] + w * (self.kappa[j] - self.kappa[i])
        v = self.v[i] + w * (self.v[j] - self.v[i])
        return TrackPoint(
            s=self.s[i] + w * (self.s[j] - self.s[i]),
            x=x,
            y=y,
            yaw=yaw,
            kappa=kappa,
            v=v,
        )


def _lerp_angle(a0: float, a1: float, w: float) -> float:
    d = wrap_angle(a1 - a0)
    return a0 + w * d


@dataclasses.dataclass
class KinematicState:
    """Robot pose + velocity feedback (what a real /odom provides)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    t: float = 0.0  # stamp (ROS time domain handled by the ROS2 layer)

    def as_pose(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw], dtype=float)


@dataclasses.dataclass
class MpcParams:
    """MPC tuning (frozen defaults; every field is config-hashed, R5).

    Control period ``Ts`` and horizon ``N`` are the two "everywhere"
    parameters: model, constraints units and reports must stay consistent
    with them (U1 freeze).
    """

    Ts: float = 0.05           # control period [s] -> 20 Hz
    N: int = 25                # prediction horizon (steps)
    Q_diag: tuple = (60.0, 10.0, 2.0, 1.0)   # [e_y, e_psi, v, omega]
    Q_F_diag: tuple = (120.0, 20.0, 4.0, 2.0)  # terminal weight
    S_diag: tuple = (0.5, 0.5)              # acceleration penalty [a, alpha]
    v_min: float = 0.0         # [m/s]  (forward only in MVP)
    v_max: float = 1.5         # [m/s]
    omega_max: float = 2.0     # [rad/s]  (symmetrical bounds)
    a_max: float = 1.0         # [m/s^2]  max linear acceleration
    alpha_max: float = 2.0     # [rad/s^2] max angular acceleration
    # Look-ahead anchoring.  Kept at 0 for the reference core: on a
    # delay-free plant it only adds a curve-cutting offset; delay
    # compensation belongs to the system-identification stage (C5), where
    # the ROS2 layer may raise it to ~v*latency.
    lookahead_m: float = 0.0
    # Solver
    qp_max_iter: int = 1500
    qp_abs_tol: float = 1e-6
    qp_rel_tol: float = 1e-5
    qp_timeout_s: float = 0.01  # 20 Hz period -> 10 ms budget (diagnostic gate)
    # Fallback
    stop_deceleration: float = 0.8   # [m/s^2] used by the degrade-to-stop fallback
    max_hold_cycles: int = 10        # how many cycles a non-critical hold may last

    def config_hash(self) -> str:
        """Deterministic hash of the whole configuration (R5)."""
        import hashlib

        payload = (
            f"Ts={self.Ts:.10g};N={self.N};Q={self.Q_diag};QF={self.Q_F_diag};"
            f"S={self.S_diag};v=[{self.v_min:.10g},{self.v_max:.10g}];"
            f"om={self.omega_max:.10g};a={self.a_max:.10g};al={self.alpha_max:.10g};"
            f"lk={self.lookahead_m:.10g};solver=admm/{self.qp_max_iter}/"
            f"{self.qp_abs_tol:.3g}/{self.qp_rel_tol:.3g}"
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


@dataclasses.dataclass
class MpcDiagnostics:
    """Per-cycle diagnostics published by the controller (R22)."""

    health: HealthState = HealthState.OK
    reason: str = ""
    qp_status: str = ""            # SOLVED / APPROXIMATE / FAILED / SKIPPED
    qp_iterations: int = 0
    qp_time_us: int = 0
    qp_objective: float = 0.0
    constraint_violation: float = 0.0   # max |violation| of hard bounds (<=0 -> none)
    clamped: bool = False          # output passed through the safety clamp
    fallback_used: bool = False
    fallback_stage: int = 0        # 0 none, 1 degrade speed, 2 zero/hold, 3 emergency
    cmd_vel: tuple = (0.0, 0.0)    # final (v_cmd, omega_cmd)
    e_ref: tuple = (0.0, 0.0, 0.0, 0.0)  # error state used this cycle


@dataclasses.dataclass
class MpcOutput:
    """One controller cycle result."""

    v_cmd: float = 0.0
    omega_cmd: float = 0.0
    diag: MpcDiagnostics = dataclasses.field(default_factory=MpcDiagnostics)
