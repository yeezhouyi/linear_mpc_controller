"""Fallback and safety contract (U4).

Deterministic degradation ladder when the QP or the input chain fails (R4/R13):

  0. OK                  -- normal operation
  1. SAFETY_CLAMPED      -- output saturated by the hard actuator bounds
  2. FALLBACK_ACTIVE     -- QP failure: speed degraded geometrically to zero
                            (a controller must NEVER keep broadcasting the
                            previous non-zero command indefinitely)
  3. EMERGENCY_STOP      -- stop time budget exceeded or a critical input
                            condition (NaN / stale state / no reference)

Any output that is not validated this cycle is zeroed first, then the
degradation policy decides how fast the command ramps to zero.
"""
from __future__ import annotations

from dataclasses import dataclass

from mpc_core.types import HealthState, MpcDiagnostics, MpcParams


@dataclass
class FallbackPolicy:
    params: MpcParams

    # internal state
    _active: bool = False
    _degrade_cycles: int = 0
    _last_v: float = 0.0

    def reset(self) -> None:
        self._active = False
        self._degrade_cycles = 0
        self._last_v = 0.0

    def apply(
        self,
        candidate: tuple,          # (v_cmd, omega_cmd) from a *successful* solve
        health: HealthState,
        diag: MpcDiagnostics,
    ) -> tuple:
        """Return the safe (v_cmd, omega_cmd) and update ``diag``."""
        v, omega = float(candidate[0]), float(candidate[1])

        # 1) NaN / non-finite: never emit garbage.
        if not (np_finite(v) and np_finite(omega)):
            diag.health = HealthState.NAN_OUTPUT
            diag.reason = "non-finite candidate"
            diag.fallback_used = True
            diag.fallback_stage = 3
            self._active = False
            self._degrade_cycles = 0
            return (0.0, 0.0)

        # 2) Critical input conditions (no/stale reference, TF/state stale,
        #    NaN, emergency): immediate safe zero -- never reuse last cmd.
        if health.is_critical:
            diag.health = health
            diag.reason = diag.reason or HealthState(health).name
            diag.fallback_used = True
            diag.fallback_stage = 3
            self._active = False
            self._degrade_cycles = 0
            return (0.0, 0.0)

        # 3) QP-level failure -> deterministic degrade-to-stop (R4/R13 ladder).
        if health in (HealthState.QP_TIMEOUT, HealthState.QP_INFEASIBLE, HealthState.FALLBACK_ACTIVE):
            if not self._active:
                self._active = True
                self._degrade_cycles = 0
                self._last_v = max(abs(v), 0.0) or abs(diag.cmd_vel[0])
            self._degrade_cycles += 1
            # geometric decay with the configured deceleration bound
            factor = max(0.0, 1.0 - self.params.stop_deceleration * self.params.Ts / max(self._last_v, 1e-6))
            v_safe = self._last_v * factor
            self._last_v = v_safe
            if self._degrade_cycles > self.params.max_hold_cycles:
                diag.health = HealthState.EMERGENCY_STOP
                diag.reason = f"stop budget exceeded after {self._degrade_cycles} cycles"
                diag.fallback_used = True
                diag.fallback_stage = 3
                self._active = False
                return (0.0, 0.0)
            diag.health = HealthState.FALLBACK_ACTIVE
            diag.reason = f"degrade-to-stop cycle {self._degrade_cycles}"
            diag.fallback_used = True
            diag.fallback_stage = 2
            return (max(v_safe, 0.0), 0.0)  # heading hold while stopping

        # 4) Healthy: clamp to hard actuator bounds.
        self._active = False
        v_c = min(max(v, self.params.v_min), self.params.v_max)
        w_c = min(max(omega, -self.params.omega_max), self.params.omega_max)
        clamped = abs(v_c - v) > 1e-12 or abs(w_c - omega) > 1e-12
        if clamped:
            diag.clamped = True
            if diag.health == HealthState.OK:
                diag.health = HealthState.SAFETY_CLAMPED
            diag.reason = "actuator bound clamp"
        return (v_c, w_c)


def np_finite(x: float) -> bool:
    import math

    return math.isfinite(x)
