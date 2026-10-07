"""Additive T07 v1 fast environment contract.

This module is deliberately independent of the legacy 12D/2D environment.
It freezes the measurement/truth split, FIFO delay, one-MPC-call step and
55D history observation needed by the training adapter.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import numpy as np

CONTRACT = "t07_v1"
OBS_FEATURES = ("e_y", "e_psi", "measured_v", "measured_omega", "ref_v",
                "ref_kappa", "e_y_dot", "e_psi_dot", "prev_final_v",
                "prev_final_omega", "mpc_health_ok")
OBS_BASE_DIM = len(OBS_FEATURES)
HISTORY_LENGTH = 5
OBS_DIM = OBS_BASE_DIM * HISTORY_LENGTH
ACTION_DIM = 1
ACTION_DELTA_OMEGA_SCALE = 0.05
SUPPORTED_PATH_IDS = ("straight", "circle", "s_curve", "u_turn",
                      "left_arc", "right_arc")
OBS_NORMALIZATION_STD = np.array(
    [1.0, 0.5, 1.0, 1.0, 1.0, 2.5, 5.0, 5.0, 1.0, 1.0, 1.0], dtype=float)


@dataclass(frozen=True)
class T07Config:
    dt_s: float = 0.05
    delay_steps: int = 0
    measurement_noise_std: float = 0.0
    encoder_bias_v: float = 0.0
    imu_bias_omega: float = 0.0
    velocity_lag: float = 0.0
    v_limit: float = 0.30
    omega_limit: float = 1.20
    path_id: str = "straight"

    def validate(self) -> None:
        vals = (self.dt_s, self.measurement_noise_std, self.encoder_bias_v,
                self.imu_bias_omega, self.velocity_lag, self.v_limit,
                self.omega_limit)
        if not all(np.isfinite(vals)):
            raise ValueError("T07 parameters must be finite")
        if self.dt_s <= 0 or self.v_limit <= 0 or self.omega_limit <= 0:
            raise ValueError("T07 positive limits required")
        if self.delay_steps not in (0, 1, 2):
            raise ValueError("delay_steps must be exactly 0, 1, or 2")
        if self.measurement_noise_std < 0 or self.velocity_lag < 0:
            raise ValueError("T07 noise and lag must be non-negative")
        if str(self.path_id).strip().lower() not in SUPPORTED_PATH_IDS:
            raise ValueError(
                f"unsupported T07 path_id {self.path_id!r}; expected one of "
                f"{', '.join(SUPPORTED_PATH_IDS)}"
            )


@dataclass
class _State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    omega: float = 0.0


class DelayQueue:
    """Causal FIFO command delay with explicit D=0/1/2 behavior."""

    def __init__(self, delay_steps: int) -> None:
        if delay_steps not in (0, 1, 2):
            raise ValueError("delay_steps must be exactly 0, 1, or 2")
        self.delay_steps = int(delay_steps)
        self.reset()

    def reset(self) -> None:
        self._queue = [np.zeros(2, dtype=float) for _ in range(self.delay_steps)]

    def push(self, command: Sequence[float]) -> np.ndarray:
        current = np.asarray(command, dtype=float).reshape(2).copy()
        self._queue.append(current)
        return self._queue.pop(0)


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class T07FastEnv:
    """Dependency-light T07 contract environment with injectable MPC."""

    def __init__(self, config: T07Config | None = None,
                 mpc: Callable[[Mapping[str, float]], Sequence[float]] | None = None):
        self.config = config or T07Config()
        self.config.validate()
        self.mpc = mpc or (lambda _measurement: (0.15, 0.0))
        self.rng = np.random.default_rng(0)
        self.truth = _State()
        self.measured = _State()
        self.queue = DelayQueue(self.config.delay_steps)
        self.history: list[np.ndarray] = []
        self.prev_final_command = np.zeros(2, dtype=float)
        self.step_count = 0
        self.mpc_call_count = 0
        self.failed = False
        self.path_id = self.config.path_id
        self._previous_error = np.zeros(2, dtype=float)

    @property
    def effective_parameters(self) -> dict:
        return {"dt_s": self.config.dt_s, "delay_steps": self.config.delay_steps,
                "measurement_noise_std": self.config.measurement_noise_std,
                "encoder_bias_v": self.config.encoder_bias_v,
                "imu_bias_omega": self.config.imu_bias_omega,
                "velocity_lag": self.config.velocity_lag,
                "action_delta_omega_scale": ACTION_DELTA_OMEGA_SCALE,
                "observation_features": list(OBS_FEATURES),
                "history_length": HISTORY_LENGTH, "observation_dim": OBS_DIM,
                "observation_normalization_std": OBS_NORMALIZATION_STD.tolist(),
                "action_dim": ACTION_DIM, "path_id": self.path_id}

    def reset(self, seed: int | None = None, *, path_id: str | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if path_id is not None:
            self.path_id = str(path_id).strip().lower()
            if self.path_id not in SUPPORTED_PATH_IDS:
                raise ValueError(
                    f"unsupported T07 path_id {path_id!r}; expected one of "
                    f"{', '.join(SUPPORTED_PATH_IDS)}"
                )
        self.truth = _State(); self.measured = _State(); self.queue.reset()
        self.prev_final_command = np.zeros(2, dtype=float)
        self.step_count = 0; self.mpc_call_count = 0; self.failed = False
        self._previous_error = np.zeros(2, dtype=float)
        base = self._observation_base()
        self.history = [base.copy() for _ in range(HISTORY_LENGTH)]
        return self._stacked_observation(), {"effective_parameters": self.effective_parameters,
                                             "path_id": self.path_id}

    def _measure(self) -> _State:
        std = self.config.measurement_noise_std
        n = self.rng.normal(0.0, std, size=5) if std else np.zeros(5)
        self.measured = _State(self.truth.x + n[0], self.truth.y + n[1],
                               _wrap(self.truth.yaw + n[2]),
                               self.truth.v + self.config.encoder_bias_v + n[3],
                               self.truth.omega + self.config.imu_bias_omega + n[4])
        return self.measured

    def _reference(self) -> tuple[float, float, float, float]:
        if self.path_id in ("circle", "left_arc"): return 0.0, 0.0, 0.15, 0.5
        if self.path_id == "right_arc": return 0.0, 0.0, 0.15, -0.5
        return 0.0, 0.0, 0.15, 0.0

    def _observation_base(self) -> np.ndarray:
        ey_ref, epsi_ref, ref_v, ref_kappa = self._reference()
        e_y, e_psi = ey_ref - self.measured.y, _wrap(epsi_ref - self.measured.yaw)
        err = np.array([e_y, e_psi], dtype=float)
        derivative = (err - self._previous_error) / self.config.dt_s
        return np.array([e_y, e_psi, self.measured.v, self.measured.omega,
                         ref_v, ref_kappa, np.clip(derivative[0], -5, 5),
                         np.clip(derivative[1], -5, 5), self.prev_final_command[0],
                         self.prev_final_command[1], 1.0 if not self.failed else 0.0])

    def _stacked_observation(self) -> np.ndarray:
        out = np.concatenate(self.history, axis=0) / np.tile(
            OBS_NORMALIZATION_STD, HISTORY_LENGTH)
        if out.shape != (OBS_DIM,) or not np.all(np.isfinite(out)):
            raise RuntimeError("T07 observation contract violated")
        return out

    def _truth_reward(self) -> float:
        ey_ref, epsi_ref, _, _ = self._reference()
        return -float(abs(ey_ref - self.truth.y) + abs(_wrap(epsi_ref - self.truth.yaw)))

    def step(self, action: Sequence[float]):
        if self.failed:
            return self._stacked_observation(), -1.0, True, {
                "absorbing": True, "reason": "failed", "mpc_calls_this_step": 0}
        raw = np.asarray(action, dtype=float).reshape(-1)
        if raw.size != ACTION_DIM or not np.all(np.isfinite(raw)):
            raise ValueError("T07 action must be one finite scalar")
        clipped = float(np.clip(raw[0], -1.0, 1.0))
        measurement = self._measure(); self.mpc_call_count += 1
        mpc_out = np.asarray(self.mpc({"x": measurement.x, "y": measurement.y,
                                       "yaw": measurement.yaw, "v": measurement.v,
                                       "omega": measurement.omega,
                                       "path_id": self.path_id}), dtype=float).reshape(-1)
        if mpc_out.size != 2 or not np.all(np.isfinite(mpc_out)):
            self.failed = True; self.prev_final_command[:] = 0.0
            self.history[-1] = self._observation_base()
            return self._stacked_observation(), -1.0, True, {
                "absorbing": True, "reason": "mpc_nonfinite", "mpc_calls_this_step": 1}
        delta = np.array([0.0, clipped * ACTION_DELTA_OMEGA_SCALE])
        final = mpc_out + delta
        final[0] = np.clip(final[0], -self.config.v_limit, self.config.v_limit)
        final[1] = np.clip(final[1], -self.config.omega_limit, self.config.omega_limit)
        if not np.all(np.isfinite(final)):
            self.failed = True; self.prev_final_command[:] = 0.0
            return self._stacked_observation(), -1.0, True, {
                "absorbing": True, "reason": "command_nonfinite", "mpc_calls_this_step": 1}
        applied = self.queue.push(final)
        if self.config.velocity_lag > 0:
            decay = np.exp(-self.config.dt_s / self.config.velocity_lag)
            self.truth.v = float(applied[0] + (self.truth.v - applied[0]) * decay)
            self.truth.omega = float(applied[1] + (self.truth.omega - applied[1]) * decay)
        else:
            self.truth.v, self.truth.omega = map(float, applied)
        self.truth.x += self.truth.v * np.cos(self.truth.yaw) * self.config.dt_s
        self.truth.y += self.truth.v * np.sin(self.truth.yaw) * self.config.dt_s
        self.truth.yaw = _wrap(self.truth.yaw + self.truth.omega * self.config.dt_s)
        self.prev_final_command = final.copy(); self.step_count += 1
        self._previous_error = self._observation_base()[:2]
        self._measure(); self.history.append(self._observation_base())
        self.history = self.history[-HISTORY_LENGTH:]
        return self._stacked_observation(), self._truth_reward(), False, {
            "absorbing": False, "reason": "running", "mpc_calls_this_step": 1,
            "action_clipped": clipped, "delta_omega": float(delta[1]),
            "final_command": tuple(float(x) for x in final),
            "applied_command": tuple(float(x) for x in applied),
            "truth_state": self.truth.__dict__.copy(),
            "measured_state": self.measured.__dict__.copy()}
