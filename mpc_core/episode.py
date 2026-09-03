"""Deterministic offline episode runner (fast env / tests / benchmarks).

Simulates one tracking episode:

    controller.compute_cycle(state) -> cmd   (every ``Ts``)
    plant.step(cmd.v, cmd.omega, Ts)         -> new state

and records everything needed for metrics and run manifests (R24).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from mpc_core.frenet import closest_point
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, MpcOutput, Trajectory, wrap_angle


@dataclass
class EpisodeResult:
    track_name: str = ""
    completed: bool = False
    done_reason: str = ""
    steps: int = 0
    t_end: float = 0.0
    arc_end: float = 0.0
    e_y: List[float] = field(default_factory=list)
    e_psi: List[float] = field(default_factory=list)
    v: List[float] = field(default_factory=list)
    omega: List[float] = field(default_factory=list)
    cmd_v: List[float] = field(default_factory=list)
    cmd_w: List[float] = field(default_factory=list)
    health: List[int] = field(default_factory=list)
    qp_status: List[str] = field(default_factory=list)
    qp_time_us: List[int] = field(default_factory=list)
    qp_iterations: List[int] = field(default_factory=list)
    constraint_violation: List[float] = field(default_factory=list)
    qp_failures: int = 0
    fallback_count: int = 0

    @property
    def e_y_rms(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.e_y)))) if self.e_y else float("nan")

    @property
    def e_y_p95(self) -> float:
        return float(np.percentile(np.abs(self.e_y), 95)) if self.e_y else float("nan")

    @property
    def e_psi_rms(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.e_psi)))) if self.e_psi else float("nan")


def run_tracking_episode(
    controller: LinearMpcController,
    plant: DifferentialDrivePlant,
    traj: Trajectory,
    track_name: str = "",
    Ts: float = 0.05,
    max_steps: int = 4000,
    complete_margin: float = 0.15,
    progress_window: int = 100,
    progress_min_gain: float = 0.1,
    max_lateral_error: float = 1.5,
    verbose: bool = False,
) -> EpisodeResult:
    res = EpisodeResult(track_name=track_name)
    arc_hist: List[float] = []
    st = plant.state
    stall_checked_at = 0

    for step in range(max_steps):
        out: MpcOutput = controller.compute_cycle(st)
        diag = out.diag
        plant.step(out.v_cmd, out.omega_cmd, Ts)
        st = plant.state

        _, _, e_y, arc = closest_point(traj, st.x, st.y)
        e_psi = wrap_angle(st.yaw - traj.sample_by_s(arc).yaw)
        res.e_y.append(e_y)
        res.e_psi.append(e_psi)
        res.v.append(st.v)
        res.omega.append(st.omega)
        res.cmd_v.append(out.v_cmd)
        res.cmd_w.append(out.omega_cmd)
        res.health.append(int(diag.health))
        res.qp_status.append(diag.qp_status)
        res.qp_time_us.append(diag.qp_time_us)
        res.qp_iterations.append(diag.qp_iterations)
        res.constraint_violation.append(diag.constraint_violation)
        res.fallback_count += 1 if diag.fallback_used else 0
        res.qp_failures += 1 if diag.qp_status == "FAILED" else 0
        arc_hist.append(arc)

        if arc >= traj.s[-1] - complete_margin:
            res.completed = True
            res.done_reason = "COMPLETED"
            break
        if abs(e_y) > max_lateral_error:
            res.done_reason = "DIVERGED"
            break
        if diag.health == HealthState.EMERGENCY_STOP:
            res.done_reason = "EMERGENCY_STOP"
            break
        # stall detection: no meaningful arc progress over the last window
        if step - stall_checked_at >= progress_window:
            window_start = max(0, len(arc_hist) - progress_window - 1)
            gained = arc_hist[-1] - arc_hist[window_start]
            if gained < progress_min_gain:
                res.done_reason = "STALL"
                break
            stall_checked_at = step

    res.steps = len(res.e_y)
    res.t_end = res.steps * Ts
    res.arc_end = arc_hist[-1] if arc_hist else 0.0
    if not res.done_reason:
        res.done_reason = "TIMEOUT"
    if verbose:
        print(
            f"[{track_name}] done={res.done_reason} steps={res.steps} "
            f"e_y_rms={res.e_y_rms:.4f} e_y_p95={res.e_y_p95:.4f} "
            f"qp_fail={res.qp_failures} fallback={res.fallback_count}"
        )
    return res
