from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple
import math
# A5.1 constants, kept numerically identical to LinearMpcController's
# private copies so the ledger and the controller agree on what "reachable"
# means.  Changing one without the other reintroduces the class of bug
# this module exists to remove.
PROJECTION_MARGIN_M = 0.02      # measurement-noise floor, per cycle
ALLOWANCE_CAP_M = 0.30          # max banked budget: bounds a single grant
ODOM_STEP_NOISE_M = 0.015       # per-step odometry noise added to the cap
@dataclass(frozen=True)
class GateDecision:
    """Outcome of one gate cycle.  All arc quantities are in metres."""
    accepted: bool
    delta_s: float            # raw_arc - baseline BEFORE this cycle
    limit: float              # margin + banked budget used for the decision
    physical_ds: float        # displacement banked this cycle (capped)
    budget: float             # budget AFTER the decision (0 on accept)
    prev_accepted_arc: float  # baseline before this cycle
    accepted_arc: float       # baseline after this cycle
class ProgressAllowanceGate:
    """Physical-displacement reachability gate (A5.1 arithmetic).
    One instance per ledger channel.  Usage::
        gate = ProgressAllowanceGate(v_max=params.v_max, Ts=Ts)
        ...
        dec = gate.step(traj, state.x, state.y, raw_arc)
        if dec.accepted:
            progress_m += max(0.0, dec.delta_s)
    NOTE: the FIRST ``step()`` call has no previous pose, so it banks
    nothing and judges against the bare margin.  At the call sites the
    robot starts at rest on the path (``raw_arc ~ baseline``) so that
    cycle accepts trivially; a caller that starts the ledger mid-path
    should pass the matching ``accepted_arc`` to the constructor.
    """
    def __init__(
        self,
        v_max: float,
        Ts: float,
        accepted_arc: float = 0.0,
        margin_m: float = PROJECTION_MARGIN_M,
        cap_m: float = ALLOWANCE_CAP_M,
        odom_step_noise_m: float = ODOM_STEP_NOISE_M,
    ) -> None:
        if v_max <= 0.0:
            raise ValueError(f"v_max must be > 0, got {v_max}")
        if Ts <= 0.0:
            raise ValueError(f"Ts must be > 0, got {Ts}")
        self._margin = float(margin_m)
        self._cap = float(cap_m)
        # A5.1(b): a localisation jump is not motion.  One step of
        # displacement is trusted only up to the kinematic limit; the
        # excess counts as ZERO, not as a remainder carried forward.
        self._step_cap = float(v_max) * float(Ts) + float(odom_step_noise_m)
        self._accepted_arc = float(accepted_arc)
        self._budget = 0.0
        self._last_pose: Optional[Tuple[float, float]] = None
    # -- read-only state ----------------------------------------------------
    @property
    def accepted_arc(self) -> float:
        """Baseline arc the gate currently trusts (monotone non-decreasing)."""
        return self._accepted_arc
    @property
    def budget(self) -> float:
        return self._budget
    @property
    def step_cap(self) -> float:
        return self._step_cap
    @property
    def margin(self) -> float:
        return self._margin
    # -- gate ---------------------------------------------------------------
    def step(self, traj, x: float, y: float, raw_arc: float) -> GateDecision:
        """Bank this cycle's motion, then judge ``raw_arc`` against it.
        ``traj`` only needs ``sample_by_s(s) -> point with .yaw``.
        """
        prev_arc = self._accepted_arc
        # 1) allowance from real motion along the path tangent (A5.1).
        #    (a) the tangent is taken at the last ACCEPTED arc, never at
        #        the raw candidate's segment: when the projection snaps to
        #        an antiparallel lane the tangent flips 180 deg, and a
        #        negative component would become positive -- the wrong
        #        projection would grant itself permission.
        #    (b) capped at v_max*Ts + odom noise (see _step_cap).
        if self._last_pose is None:
            physical_ds = 0.0
        else:
            dx = float(x) - self._last_pose[0]
            dy = float(y) - self._last_pose[1]
            anchor = traj.sample_by_s(prev_arc)
            projected = dx * math.cos(anchor.yaw) + dy * math.sin(anchor.yaw)
            physical_ds = min(max(0.0, projected), self._step_cap)
        # Banked BEFORE the decision: the current displacement must be part
        # of the current limit, otherwise delta_s stays one cycle ahead
        # forever whenever v*Ts > margin (the STALL failure mode).
        self._budget = min(self._budget + physical_ds, self._cap)
        self._last_pose = (float(x), float(y))
        # 2) decision.  Baseline is the ACCEPTED arc, never the last raw one.
        delta_s = float(raw_arc) - prev_arc
        limit = self._margin + self._budget
        accepted = delta_s <= limit
        if accepted:
            self._budget = 0.0
            if raw_arc > self._accepted_arc:
                self._accepted_arc = float(raw_arc)
        return GateDecision(
            accepted=accepted,
            delta_s=delta_s,
            limit=limit,
            physical_ds=physical_ds,
            budget=self._budget,
            prev_accepted_arc=prev_arc,
            accepted_arc=self._accepted_arc,
        )
    def rebaseline(self, arc: float) -> None:
        """Capture-resolve re-anchor (A5.2), FORWARD ONLY.
        Called when the anchor jumped but the robot is spatially AT the new
        anchor: the bypassed span was physically cut, so the caller books it
        as ``skipped_arc_m`` once and the gate re-baselines.  Like an accept,
        this spends the budget.
        """
        if arc > self._accepted_arc:
            self._accepted_arc = float(arc)
        self._budget = 0.0
    def reset(self, accepted_arc: float = 0.0) -> None:
        self._accepted_arc = float(accepted_arc)
        self._budget = 0.0
        self._last_pose = None
