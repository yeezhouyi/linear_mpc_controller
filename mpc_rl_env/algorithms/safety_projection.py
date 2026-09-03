"""Safety projection for residual actions (R18/U10).

``u_safe = Projection(u_mpc + alpha * delta_u_rl)`` with amplitude limits and
accel/rate shaping.  The *runtime* authority stays in the C++ core; this
module is the golden-vector reference used by tests (plan U10 safety split).
"""
from __future__ import annotations

import numpy as np


class SafetyProjection:
    def __init__(
        self,
        v_min: float = 0.0,
        v_max: float = 1.5,
        omega_max: float = 2.0,
        dv_max_per_step: float = 0.05,   # 1.0 m/s^2 * Ts
        dw_max_per_step: float = 0.1,    # 2.0 rad/s^2 * Ts
    ) -> None:
        self.limits = (v_min, v_max, omega_max)
        self.rate = (dv_max_per_step, dw_max_per_step)

    def project(self, u_mpc: np.ndarray, delta_rl: np.ndarray, alpha: float,
                prev_cmd: np.ndarray) -> np.ndarray:
        """u_mpc/delta_rl/prev_cmd are [v, omega]. Returns projected [v, omega]."""
        raw = u_mpc + alpha * delta_rl
        # amplitude
        v = np.clip(raw[0], self.limits[0], self.limits[1])
        w = np.clip(raw[1], -self.limits[2], self.limits[2])
        # rate limit vs previous command
        dv = np.clip(v - prev_cmd[0], -self.rate[0], self.rate[0])
        dw = np.clip(w - prev_cmd[1], -self.rate[1], self.rate[1])
        out = np.array([prev_cmd[0] + dv, prev_cmd[1] + dw])
        # re-clip after rate shaping (numerical)
        out[0] = np.clip(out[0], self.limits[0], self.limits[1])
        out[1] = np.clip(out[1], -self.limits[2], self.limits[2])
        return out
