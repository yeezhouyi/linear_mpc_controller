"""Deterministic offline episode runner (fast env / tests / benchmarks).

Simulates one tracking episode:

    controller.compute_cycle(state) -> cmd   (every ``Ts``)
    plant.step(cmd.v, cmd.omega, Ts)         -> new state

Progress accounting is the A5.2 accepted-arc HIGH-WATERMARK (never a raw-arc
accumulation): ``completed`` / STALL read ``arc_high_watermark``, which is
fed by the shared ``mpc_core.progress_gate.ProgressAllowanceGate`` ledger
(one ruler for control and metrics; the controller carries the A5.1 gate in
the control chain, this module audits the rollout with the same arithmetic).
Rejected projections are booked as jump EPISODES (counted once) and, when
the robot is spatially at the jumped anchor (<= capture radius), the bypassed
span is booked as ``skipped_arc_m`` once and the gate re-baselines.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from mpc_core.frenet import closest_point
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.progress_gate import ProgressAllowanceGate
from mpc_core.types import HealthState, MpcOutput, Trajectory, wrap_angle


@dataclass
class EpisodeResult:
    track_name: str = ""
    completed: bool = False
    done_reason: str = ""
    steps: int = 0
    t_end: float = 0.0
    arc_end: float = 0.0
    # ---- A5.2 accepted-arc progress (reachability-based) ----
    progress_m: float = 0.0          # sum of accepted forward delta_s
    skipped_arc_m: float = 0.0       # bypassed span booked on capture-resolve
    progress_ratio: float = 0.0      # min(progress_m / path_length, 1)
    skipped_arc_ratio: float = 0.0   # skipped_arc_m / path_length
    projection_jump_count: int = 0   # jump EPISODES (not cycles)
    projection_reacquire_count: int = 0
    arc_high_watermark: float = 0.0
    # legacy diagnostics (kept, NOT gates)
    progress_frac: float = 0.0       # legacy sum(max(0, darc))/L - can exceed 1
    max_arc_jump: float = 0.0        # largest single-step raw anchor move
    arc_end_frac: float = 0.0
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
    audit_mode: str = "accepted",
) -> EpisodeResult:
    """audit_mode: "accepted" (default) routes the ledger through the shared
    ProgressAllowanceGate (A5.2); "raw" judges progress/completion on the raw
    projection arc with NO gate -- the historically published (lying) ruler.
    "raw" exists ONLY for the A8 replication rows, which reproduce the old
    published numbers to show what they lied about; it must never be used to
    accept or reject a matrix cell (A8.3)."""
    res = EpisodeResult(track_name=track_name)
    arc_hist: List[float] = []
    st = plant.state
    stall_checked_at = 0
    stall_mark = 0.0            # A5.2: watermark at last stall check
    s_prev: Optional[float] = None
    in_jump_episode: bool = False
    projection_capture_m: float = 0.30   # robot-at-anchor capture radius
    # A5.2/A7.3: the audit ledger routes through the shared gate -- the
    # allowance is banked PHYSICAL displacement projected on the tangent at
    # the last ACCEPTED arc, never v*cos(e_psi)*Ts (measured at the raw
    # candidate, so a wrong lane would pay for itself).
    # Seed the ledger with the robot's INITIAL projection arc so a rollout
    # that legitimately starts mid-path does not book its own first cycle as
    # a jump -- parity with the controller, which anchors on its first cycle.
    # (Local adaptation, recorded: the docx episode patch leaves the module
    # default accepted_arc=0.0, which only matches runs started at arc ~ 0.)
    _, _, _, seed_arc, _ = closest_point(traj, st.x, st.y)
    gate = None
    if audit_mode != "raw":
        gate = ProgressAllowanceGate(v_max=controller.params.v_max, Ts=Ts,
                                     accepted_arc=float(seed_arc))
        # Prime the ledger with the initial pose: the module's first step()
        # has no previous pose and judges against the bare margin, but by the
        # time the first in-loop projection runs the plant has already
        # advanced one control period (v*Ts), which would trip the margin by
        # a hair.  (Local adaptation, recorded.)  Anchoring here makes the
        # first real step judge normally.
        gate.step(traj, st.x, st.y, float(seed_arc))
    path_len = float(traj.s[-1]) if len(traj.s) else 0.0

    for step in range(max_steps):
        out: MpcOutput = controller.compute_cycle(st)
        diag = out.diag
        plant.step(out.v_cmd, out.omega_cmd, Ts)
        st = plant.state

        _, _, e_y, arc, stage = closest_point(
            traj, st.x, st.y, yaw=st.yaw, s_prev=s_prev)
        if stage == 3:
            # projection ambiguous / heading gate rejected everything: stop
            # this episode honestly (A2/A4) -- no arbitrary fallback.
            res.done_reason = "PROJECTION_AMBIGUOUS"
            break
        s_prev = arc
        e_psi = wrap_angle(st.yaw - traj.sample_by_s(arc).yaw)

        # ---- A5.2 reachability-gated progress (capture-resolve) ---------
        if audit_mode == "raw":
            # Replication ruler (A8.3): the raw arc IS the progress -- no
            # gate, no capture-resolve, no jump booking.  Reproduces the
            # historical measurement exactly as it was (wrongly) done.
            pass
        else:
            dec = gate.step(traj, st.x, st.y, arc)
            anchor_pt = traj.sample_by_s(arc)
            near_anchor = math.hypot(st.x - anchor_pt.x, st.y - anchor_pt.y) <= projection_capture_m
            if not dec.accepted and near_anchor:
                if not in_jump_episode:
                    res.projection_jump_count += 1
                    in_jump_episode = True
                # the bypassed span was physically cut: book it once (re-baseline
                # makes the next cycle's delta_s small), then re-anchor FORWARD
                # ONLY (module spends the budget like an accept).
                res.skipped_arc_m += dec.delta_s
                gate.rebaseline(arc)
            elif not dec.accepted:
                if not in_jump_episode:
                    res.projection_jump_count += 1
                    in_jump_episode = True
                # unresolved: a returning anchor resumes crediting without
                # double-booking the return lane (no re-baseline here).
            else:
                in_jump_episode = False
                if dec.delta_s > 0.0:
                    res.progress_m += dec.delta_s
        res.arc_high_watermark = max(
            res.arc_high_watermark,
            arc if audit_mode == "raw" else gate.accepted_arc)

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
        # A4.2 reacquire events as reported by the controller's own counter
        res.projection_reacquire_count = diag.reacquire_count
        arc_hist.append(arc)

        # A5.3: completion reads the HIGH-WATERMARK of accepted arc, never the
        # raw arc (the raw value caused the historical false-COMPLETED); and
        # completion is REFUSED while the controller is in reacquire probation.
        if (res.arc_high_watermark >= traj.s[-1] - complete_margin
                and not diag.in_probation):
            res.completed = True
            res.done_reason = "COMPLETED"
            break
        if abs(e_y) > max_lateral_error:
            res.done_reason = "DIVERGED"
            break
        if diag.health == HealthState.EMERGENCY_STOP:
            # A4.2: PROJECTION_LOST / PROJECTION_AMBIGUOUS are their own
            # terminal reasons; conflating them with a generic emergency
            # hides the projection-chain failure in the matrix.
            reason = diag.reason or ""
            if "PROJECTION_LOST" in reason:
                res.done_reason = "PROJECTION_LOST"
            elif "PROJECTION_AMBIGUOUS" in reason:
                res.done_reason = "PROJECTION_AMBIGUOUS"
            else:
                res.done_reason = "EMERGENCY_STOP"
            break
        # stall detection: no ACCEPTED-arc progress over the last window
        if step - stall_checked_at >= progress_window:
            gained = res.arc_high_watermark - stall_mark
            if gained < progress_min_gain:
                res.done_reason = "STALL"
                break
            stall_mark = res.arc_high_watermark
            stall_checked_at = step

    res.steps = len(res.e_y)
    res.t_end = res.steps * Ts
    res.arc_end = arc_hist[-1] if arc_hist else 0.0
    if len(arc_hist) >= 2 and traj.s[-1] > 1e-9:
        d = np.diff(np.asarray(arc_hist))
        res.progress_frac = float(np.sum(np.maximum(0.0, d)) / traj.s[-1])
        res.max_arc_jump = float(np.max(d))
        res.arc_end_frac = float(res.arc_end / traj.s[-1])
    # A5.2 ratios (cannot exceed 1 by construction)
    if path_len > 1e-9:
        res.progress_ratio = min(res.progress_m / path_len, 1.0)
        res.skipped_arc_ratio = min(res.skipped_arc_m / path_len, 1.0)
    if not res.done_reason:
        res.done_reason = "TIMEOUT"
    if verbose:
        print(
            f"[{track_name}] done={res.done_reason} steps={res.steps} "
            f"prog={res.progress_ratio:.3f} skip={res.skipped_arc_ratio:.4f} "
            f"jumps={res.projection_jump_count} "
            f"e_y_rms={res.e_y_rms:.4f} qp_fail={res.qp_failures} "
            f"fallback={res.fallback_count}"
        )
    return res
