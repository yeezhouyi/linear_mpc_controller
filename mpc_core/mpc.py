"""Condensed-QP linear MPC trajectory-tracking controller (U3).

Implements the receding-horizon controller for the frozen model

    x = [e_y, e_psi, v, omega],   u = [a, alpha]  (accel inputs)

with the discrete LTV prediction ``x[k+1] = A_d[k] x[k] + B_d[k] u[k]``,
condensed into a dense QP over the stacked input sequence.  Constraints:

* velocity bounds   ``v_min <= v_k <= v_max``  (hard, over the horizon),
* angular bounds    ``|omega_k| <= omega_max``,
* accel bounds      ``|a_k| <= a_max``,  ``|alpha_k| <= alpha_max``,
* (soft constraint relaxation is a documented follow-up, C2/DoD).

The first predicted velocity is the ``cmd_vel`` candidate; failures degrade
through :class:`FallbackPolicy`.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

from mpc_core.fallback import FallbackPolicy
from mpc_core.frenet import closest_point, frenet_error_at_arc
from mpc_core.model import build_ltv_window, reference_state_vector
from mpc_core.qp import AdmmQp
from mpc_core.types import (
    A_IDX,
    ALPHA_IDX,
    EY_IDX,
    EPSI_IDX,
    NX,
    NU,
    OMEGA_IDX,
    V_IDX,
    HealthState,
    KinematicState,
    MpcDiagnostics,
    MpcOutput,
    MpcParams,
    Trajectory,
)

INF = float("inf")


class LinearMpcController:
    """Receding-horizon linear MPC over a reference ``Trajectory``."""

    def __init__(self, params: MpcParams, traj: Optional[Trajectory] = None) -> None:
        self.params = params
        self.traj = traj
        self.fallback = FallbackPolicy(params)
        self.solver = AdmmQp(
            max_iter=params.qp_max_iter,
            abs_tol=params.qp_abs_tol,
            rel_tol=params.qp_rel_tol,
        )
        self._warm: Optional[np.ndarray] = None
        self.cycle = 0
        # ---- A5.1 acceptance-gate state (single gate, in chain) ----
        # baseline: last ACCEPTED arc (window centre + delta_s baseline),
        # forward-only; budget: banked PHYSICAL displacement; reject_run:
        # consecutive rejections -> A4.2 reacquire.
        self._baseline: Optional[float] = None
        self._last_xy: Optional[Tuple[float, float]] = None
        self._budget: float = 0.0
        self._reject_run: int = 0
        # ---- A4.2 reacquire protocol state --------------------------
        self._reacquire_mode = "NORMAL"  # NORMAL | SEEKING | PROBATION
        self._seeking_steps: int = 0
        self._stable_run: int = 0
        self._stable_seg: int = -1
        self._probation_left: int = 0
        self._reacquire_events: int = 0

    # -- public API ---------------------------------------------------------

    def set_reference(self, traj: Optional[Trajectory]) -> None:
        self.traj = traj
        self.fallback.reset()
        self._warm = None
        self._baseline = None
        self._last_xy = None
        self._budget = 0.0
        self._reject_run = 0
        self._reacquire_mode = "NORMAL"
        self._seeking_steps = 0
        self._stable_run = 0
        self._stable_seg = -1
        self._probation_left = 0

    def compute_cycle(self, state: KinematicState) -> MpcOutput:
        """One controller cycle at period ``Ts``. Pure core: no ROS time/TF
        checks here (those live in the ROS2 node); inputs are assumed fresh.
        """
        self.cycle += 1
        out = MpcOutput()
        diag = out.diag

        if self.traj is None:
            diag.health = HealthState.NO_REFERENCE
            diag.reason = "no reference set"
            diag.fallback_used = True
            diag.fallback_stage = 3
            return out

        p = self.params
        # ---- A4.2 reacquire seeking: no QP while we re-anchor -------
        if self._reacquire_mode == "SEEKING":
            self._seeking_steps += 1
            if self._seeking_steps > p.reacquire_timeout_steps:
                diag.health = HealthState.EMERGENCY_STOP
                diag.reason = "PROJECTION_LOST"
                diag.fallback_used = True
                diag.fallback_stage = 3
                diag.in_probation = True
                return out
            seg, _, _, cand_arc, cand_stage = closest_point(
                self.traj, state.x, state.y, yaw=state.yaw)
            if cand_stage == 3:
                self._stable_run = 0
                self._stable_seg = -1
            else:
                if seg == self._stable_seg:
                    self._stable_run += 1
                else:
                    self._stable_run, self._stable_seg = 1, seg
                if self._stable_run >= p.reacquire_stable_steps:
                    # atomic commit (A4.2 step 4/5): new baseline,
                    # zero budget, drop QP warm start, then probation.
                    self._baseline = float(cand_arc)
                    self._budget = 0.0
                    self._reject_run = 0
                    self._last_xy = (float(state.x), float(state.y))
                    self._warm = None
                    self._reacquire_mode = "PROBATION"
                    self._probation_left = p.probation_steps
                    self._reacquire_events += 1
            diag.health = HealthState.OK
            diag.reason = "REACQUIRE_SEEKING"
            diag.reacquire_count = self._reacquire_events
            diag.accepted_arc = float(self._baseline or 0.0)
            return out  # decel-to-zero: zero command while seeking
        # ---- raw projection for the gate (window centred on baseline)
        seg, _, _, arc, stage = closest_point(
            self.traj, state.x, state.y, yaw=state.yaw,
            s_prev=self._baseline)
        if stage == 3:
            # A2: heading gate rejected every candidate / projection
            # ambiguous.  NO unconstrained global fallback -- that was
            # the original fail-open defect.  Stop BEFORE any QP input
            # is built (A3.2); baseline stays frozen.
            diag.health = HealthState.EMERGENCY_STOP
            diag.reason = "PROJECTION_AMBIGUOUS"
            diag.fallback_used = True
            diag.fallback_stage = 3
            return out
        # ---- A5.1 acceptance gate (unique state machine) -------------
        if self._baseline is None:
            # anchor cycle: nothing to compare against; accept the first
            # trustworthy projection (module semantics: first step banks
            # nothing and judges against the bare margin).
            self._baseline = float(arc)
            self._reject_run = 0
        else:
            prev = self._baseline
            # 1) bank PHYSICAL displacement along the tangent at the last
            #    ACCEPTED arc (never the raw candidate's tangent).
            if self._last_xy is None:
                physical_ds = 0.0
            else:
                dx = float(state.x) - self._last_xy[0]
                dy = float(state.y) - self._last_xy[1]
                tan_pt = self.traj.sample_by_s(prev)
                projected = dx * math.cos(tan_pt.yaw) + dy * math.sin(tan_pt.yaw)
                step_cap = p.v_max * p.Ts + p.odom_step_noise_m
                physical_ds = min(max(0.0, projected), step_cap)
            self._budget = min(self._budget + physical_ds, p.allowance_cap_m)
            delta_s = float(arc) - prev
            limit = p.projection_margin_m + self._budget
            if delta_s < -p.back_m:
                self._reject_run += 1          # backward-lane jump
            elif delta_s <= limit:
                self._budget = 0.0              # spent on accept
                self._reject_run = 0
                if arc > prev:
                    self._baseline = float(arc)  # forward-only
            else:
                self._reject_run += 1          # forward jump over limit
            if self._reject_run > p.max_reject_run:
                # A4.2: stop trusting the stale anchor; decel and seek.
                self._reacquire_mode = "SEEKING"
                self._seeking_steps = 0
                self._stable_run = 0
                self._stable_seg = -1
        self._last_xy = (float(state.x), float(state.y))
        # ---- anchor/err at the ACCEPTED arc (A5.0/A3.1) --------------
        anchor, err = frenet_error_at_arc(
            self.traj, state, self._baseline, p.lookahead_m)
        diag.e_ref = tuple(float(e) for e in err)
        diag.accepted_arc = float(self._baseline or 0.0)
        diag.reacquire_count = self._reacquire_events
        if self._reacquire_mode == "PROBATION":
            self._probation_left -= 1
            if self._probation_left <= 0:
                self._reacquire_mode = "NORMAL"

        # ---- build prediction window -----------------------------------
        As, Bs, anchors = build_ltv_window(
            self.traj, anchor.s, self.params.Ts, self.params.N
        )
        x0 = err

        # ---- build condensed QP ----------------------------------------
        P, q, C, cl, cu = self._build_condensed_qp(x0, As, Bs, anchors)

        # ---- solve (warm-started) --------------------------------------
        t0 = time.perf_counter()
        res = self.solver.solve(P, q, C, cl, cu, warm_start=self._warm)
        diag.qp_time_us = res.solve_time_us
        diag.qp_iterations = res.iterations
        diag.qp_objective = res.objective
        diag.qp_status = res.status
        del t0

        # ---- extract command / health ----------------------------------
        if res.ok:
            U = res.x
            self._warm = np.roll(U, -NU)  # shift for next cycle
            self._warm[-NU:] = 0.0
            a0, alpha0 = float(U[A_IDX]), float(U[NU + ALPHA_IDX])
            v_cmd = err[V_IDX] + self.params.Ts * a0
            w_cmd = err[OMEGA_IDX] + self.params.Ts * alpha0
            diag.health = HealthState.OK
            diag.reason = ""
            # measure constraint satisfaction of the *applied* plan
            diag.constraint_violation = self._max_violation(x0, As, Bs, anchors, U)
            if res.status == "APPROXIMATE":
                diag.reason = "approximate QP solve"
            # A4.2 probation / reject-run speed limit (A5.0)
            if self._reacquire_mode == "PROBATION" or self._reject_run > 0:
                v_cmd = min(v_cmd, p.v_probation)
            diag.in_probation = self._reacquire_mode == "PROBATION"
            v_safe, w_safe = self.fallback.apply((v_cmd, w_cmd), HealthState.OK, diag)
            # keep the degraded health if the fallback changed anything
            if diag.health != HealthState.OK:
                pass
            diag.cmd_vel = (v_safe, w_safe)
            out.v_cmd, out.omega_cmd = v_safe, w_safe
            return out

        # ---- solver failure: deterministic fallback ---------------------
        reason = "qp failed"
        health = HealthState.QP_INFEASIBLE
        if not np.all(np.isfinite(q)):
            reason = "non-finite cost"
            health = HealthState.NAN_OUTPUT
        v_safe, w_safe = self.fallback.apply((0.0, 0.0), health, diag)
        diag.health = health
        diag.reason = reason
        diag.fallback_used = True
        diag.cmd_vel = (v_safe, w_safe)
        out.v_cmd, out.omega_cmd = v_safe, w_safe
        return out

    # -- internals ----------------------------------------------------------

    def _build_condensed_qp(self, x0, As, Bs, anchors):
        """Return ``(P, q, C, cl, cu)`` for the dense ADMM solver."""
        p = self.params
        N = p.N
        # prediction matrices F (4N x 4), G (4N x 2N)
        F = np.zeros((NX * N, NX))
        G = np.zeros((NX * N, NU * N))
        F[0:NX] = As[0]
        G[0:NX, 0:NU] = Bs[0]
        for k in range(1, N):
            F[k * NX:(k + 1) * NX] = As[k] @ F[(k - 1) * NX:k * NX]
            G[k * NX:(k + 1) * NX, 0:k * NU] = As[k] @ G[(k - 1) * NX:k * NX, 0:k * NU]
            G[k * NX:(k + 1) * NX, k * NU:(k + 1) * NU] = Bs[k]

        # weights
        Q = np.diag(p.Q_diag)
        QF = np.diag(p.Q_F_diag)
        S = np.diag(p.S_diag)
        Qbar = np.zeros((NX * N, NX * N))
        for k in range(N):
            blk = QF if k == N - 1 else Q
            Qbar[k * NX:(k + 1) * NX, k * NX:(k + 1) * NX] = blk
        Sbar = np.zeros((NU * N, NU * N))
        for k in range(N):
            Sbar[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] = S

        # reference stack for predicted states x_1..x_N -> anchors[1..N]
        Xref = np.zeros(NX * N)
        for k in range(N):
            Xref[k * NX:(k + 1) * NX] = reference_state_vector(anchors[k + 1])

        x_free = F @ x0 - Xref
        P_mat = G.T @ Qbar @ G + Sbar
        q_vec = G.T @ Qbar @ x_free

        # ---- constraints: rows l <= C u <= u ----------------------------
        rows_c, rows_l, rows_u = [], [], []
        n_states_viol = 0
        for k in range(N):
            Fk = F[k * NX:(k + 1) * NX]          # 4 x 4
            Gk = G[k * NX:(k + 1) * NX]          # 4 x 2N
            # v upper bound: e_v x_k <= v_max
            c_plus = Gk[V_IDX]
            rows_c.append(c_plus)
            rows_l.append(-INF)
            rows_u.append(p.v_max - float(Fk[V_IDX] @ x0))
            # v lower bound: -e_v x_k <= -v_min
            rows_c.append(-c_plus)
            rows_l.append(-INF)
            rows_u.append(-p.v_min + float(Fk[V_IDX] @ x0))
            # omega bounds
            c_om = Gk[OMEGA_IDX]
            rows_c.append(c_om)
            rows_l.append(-INF)
            rows_u.append(p.omega_max - float(Fk[OMEGA_IDX] @ x0))
            rows_c.append(-c_om)
            rows_l.append(-INF)
            rows_u.append(p.omega_max + float(Fk[OMEGA_IDX] @ x0))
        # input accel bounds: |a_k| <= a_max, |alpha_k| <= alpha_max
        for k in range(N):
            for (idx, lim) in ((0, p.a_max), (1, p.alpha_max)):
                e = np.zeros(NU)
                e[idx] = 1.0
                col = k * NU + idx
                row = np.zeros(NU * N)
                row[col] = 1.0
                rows_c.append(row)
                rows_l.append(-INF)
                rows_u.append(lim)
                rows_c.append(-row)
                rows_l.append(-INF)
                rows_u.append(lim)
        C = np.array(rows_c) if rows_c else np.zeros((0, NU * N))
        cl = np.array(rows_l) if rows_l else np.zeros(0)
        cu = np.array(rows_u) if rows_u else np.zeros(0)
        return P_mat, q_vec, C, cl, cu

    def _max_violation(self, x0, As, Bs, anchors, U) -> float:
        p = self.params
        xs = np.zeros((p.N + 1, NX))
        xs[0] = x0
        for k in range(p.N):
            xs[k + 1] = As[k] @ xs[k] + Bs[k] @ U[k * NU:(k + 1) * NU]
        viol = 0.0
        for k in range(1, p.N + 1):
            v = xs[k, V_IDX]
            w = xs[k, OMEGA_IDX]
            viol = max(viol, p.v_min - v, v - p.v_max, abs(w) - p.omega_max)
        return float(max(viol, 0.0))
