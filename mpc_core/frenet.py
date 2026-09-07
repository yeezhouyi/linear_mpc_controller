"""Frenet projection and error extraction (U2).

Given a robot pose and a dense reference ``Trajectory``, compute

* the reference anchor point (closest point projection with optional
  look-ahead), and
* the signed error state ``[e_y, e_psi, v, omega]``.

Sign conventions (frozen, see docs/mpc_model_derivation.md):
  ``e_y > 0`` means the robot is LEFT of the reference travel direction;
  ``e_psi = wrap(psi - psi_ref)``.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from mpc_core.types import Trajectory, TrackPoint, KinematicState, wrap_angle


def _segments(traj: Trajectory) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Segment start/end points, tangent unit vectors, lengths."""
    dx = np.diff(traj.x)
    dy = np.diff(traj.y)
    lengths = np.hypot(dx, dy)
    # Degenerate segments (duplicate points) get tangent = previous tangent.
    tangents = np.stack(
        [
            np.where(lengths > 1e-12, dx / np.maximum(lengths, 1e-12), 0.0),
            np.where(lengths > 1e-12, dy / np.maximum(lengths, 1e-12), 0.0),
        ],
        axis=-1,
    )
    # fix zero tangents (straight repeats) by carrying forward
    for i in range(1, tangents.shape[0]):
        nz = np.hypot(tangents[i, 0], tangents[i, 1]) < 1e-12
        if nz:
            tangents[i] = tangents[i - 1]
    return dx, dy, lengths, tangents


def closest_point(
    traj: Trajectory,
    px: float,
    py: float,
    yaw: float | None = None,
    travel_sign: int | None = None,   # test-only uniform-gear override, see A2.2
    s_prev: float | None = None,
    back_m: float = 0.08,
    fwd_m: float = 0.30,
    reacquire_m: float = 1.00,
    wide_m: float = 5.00,
    heading_gate_rad: float = 1.5708,   # pi/2
    tie_eps_m: float = 0.05,
) -> Tuple[int, float, float, float, int]:
    """Nearest-segment projection with an arc window and a heading gate.
    ``s_prev is None`` reproduces the original stateless global search, so
    the determinism tests keep their meaning.  With ``s_prev`` given the
    search escalates in three stages:
      0. narrow window ``[s_prev - back_m, s_prev + fwd_m]`` -- nominal;
      1. wide window ``[s_prev +- wide_m]``                  -- disturbance;
      2. global                                              -- kidnapped.
    ``yaw`` enables the heading gate: candidate segments whose tangent
    differs from the robot heading by more than ``heading_gate_rad`` are
    rejected at EVERY stage.  This is the cheapest structural defence
    against snapping onto an adjacent antiparallel lane and it keeps
    working even at stage 2, where the plain global nearest point can
    itself be the wrong lane.
    Ties (candidates within ``tie_eps_m`` of one another) resolve toward
    the one nearest ``s_prev`` in arc length.  With no ``s_prev`` and a
    genuine tie the projection is ambiguous: ``stage`` is returned as 3
    and the caller must stop rather than pick arbitrarily.
    ``stage == 3`` is also returned when the heading gate rejects every
    segment.  There is deliberately NO unconstrained fallback: a gate that
    can be bypassed when it rejects everything is not a gate.
    Returns ``(seg_idx, w, e_y, arc, stage)``.
    """
    dx, dy, lengths, tangents = _segments(traj)
    rel_x = px - traj.x[:-1]
    rel_y = py - traj.y[:-1]
    tx = tangents[:, 0]
    ty = tangents[:, 1]
    along = rel_x * tx + rel_y * ty
    along_c = np.clip(along, 0.0, lengths)
    proj_x = traj.x[:-1] + along_c * tx
    proj_y = traj.y[:-1] + along_c * ty
    dist2 = (proj_x - px) ** 2 + (proj_y - py) ** 2
    # heading gate: reject segments pointing the wrong way.
    #
    # A REVERSE connector is driven with the body pointing OPPOSITE the path
    # tangent, so comparing yaw against the tangent directly would reject
    # every legitimate reverse segment.  Compare the MOTION direction
    # instead.  Direction of travel is read from the EXPLICIT per-segment
    # ``traj.segment_gear`` (strictly +-1, never inferred from ``traj.v``:
    # at a cusp the reference speed is exactly zero and the sign is
    # meaningless).  ``travel_sign`` is a TEST-ONLY override that forces one
    # uniform gear over the whole path; it must not be used on a path that
    # mixes forward and reverse segments.  See A2.2.
    ok = np.ones(dist2.shape, dtype=bool)
    if yaw is not None:
        seg_yaw = np.arctan2(ty, tx)
        if travel_sign is not None:
            sgn = np.full(seg_yaw.shape, -1.0 if travel_sign < 0 else 1.0)
        else:
            # segment_gear is EXPLICIT and strictly +-1 per segment; never
            # inferred from sign(traj.v).  See A2.2.
            sgn = np.asarray(traj.segment_gear, dtype=float)
        motion_yaw = np.where(sgn > 0.0, yaw, yaw + np.pi)
        dpsi = np.abs(np.arctan2(np.sin(motion_yaw - seg_yaw),
                                 np.cos(motion_yaw - seg_yaw)))
        ok = dpsi <= heading_gate_rad
    def _pick(idx: np.ndarray) -> int:
        """argmin over idx, ties broken toward s_prev; -1 if none."""
        if idx.size == 0:
            return -1
        d = dist2[idx]
        best = float(d.min())
        thr = (np.sqrt(best) + tie_eps_m) ** 2
        m = d <= thr
        near = idx[m]
        if near.size > 1:
            # RECORDED DEVIATION (交接文档 R5/R6): on dense (0.02 m) sampling
            # the eps band always contains the same-lane micro-segments around
            # the foot (their clamped feet lie within tie_eps of the best
            # distance), so the raw doc tie logic keeps picking the candidate
            # nearest s_prev -- an arc lag that never clears.  Candidates on
            # the SAME lane (tangent within 0.05 rad) collapse to the single
            # best one; only genuinely different-lane ties (antiparallel or
            # neighbouring lanes) reach the s_prev / stage-3 path.
            j = int(np.argmin(d[m]))
            # Same-PLACE collapse first: if every near candidate's foot lies
            # within 2*tie_eps of the best foot they are the SAME physical
            # place (a closed path's head/tail seam, both strands arriving at
            # the same vertex).  Return the best.
            feet_x = proj_x[near] - proj_x[near[j]]
            feet_y = proj_y[near] - proj_y[near[j]]
            if np.all(np.hypot(feet_x, feet_y) <= 2.0 * tie_eps_m):
                return int(near[j])
            # ARC-CONTIGUITY run partition (robust on curved paths: the
            # admitted band on a circle spans enough arc for the tangent to
            # rotate >0.05 rad WITHIN one lane, so tangent difference is the
            # wrong discriminator).  One contiguous run = one lane -> the
            # argmin best.  Several runs separated by an arc gap = different
            # places/lanes -> genuine no-s_prev ambiguity below.
            arcs_n = traj.s[near] + along_c[near]
            order = np.argsort(arcs_n)
            sorted_arcs = arcs_n[order]
            gap = np.diff(sorted_arcs)
            run_breaks = np.where(gap > max(0.05, 4.0 * (sorted_arcs[-1] - sorted_arcs[0]) / max(near.size - 1, 1)))[0]
            # simpler: a break is any inter-candidate arc gap > 5x the median
            # intra-run spacing, floored at 0.05 m
            med = float(np.median(gap)) if gap.size else 0.0
            breaks = gap > max(0.05, 5.0 * med)
            if not np.any(breaks):
                return int(near[j])
            # Multiple arc runs.  They are a genuine no-state ambiguity ONLY
            # when they are DIFFERENT directions (antiparallel / crossing
            # lanes).  A closed path's seam puts its head and tail runs at
            # the same place with the SAME travel direction (full-loop start
            # tangent ~= end tangent): there the deterministic argmin is the
            # correct anchor/observation answer and matches the doc's
            # "stateless reproduces the original search".
            sy = np.arctan2(ty, tx)
            run_starts = np.concatenate(([0], np.where(breaks)[0] + 1))
            reps = []
            rep_angles = []
            for rs in run_starts:
                run_idx = near[order[rs:]]
                b = int(np.argmin(dist2[run_idx]))
                reps.append(run_idx[b])
                rep_angles.append(float(sy[run_idx[b]]))
            dirs = np.asarray(rep_angles)
            dpsi = np.abs((dirs[:, None] - dirs[None, :] + math.pi)
                          % (2.0 * math.pi) - math.pi)
            if np.max(dpsi) <= 0.52:   # ~30 deg: same direction -> argmin
                return int(near[j])
            near = np.asarray(reps, dtype=int)
            d = dist2[near]
        if near.size > 1 and s_prev is not None:
            arcs = traj.s[near] + along_c[near]
            return int(near[int(np.argmin(np.abs(arcs - float(s_prev))))])
        if near.size > 1:
            return -2          # ambiguous, no state to break the tie
        return int(near[0])
    seg, stage = -1, 2
    if s_prev is not None:
        r2 = reacquire_m * reacquire_m
        for st, (lo_m, hi_m) in enumerate(((back_m, fwd_m), (wide_m, wide_m))):
            lo = float(s_prev) - lo_m
            hi = float(s_prev) + hi_m
            in_win = (traj.s[1:] >= lo) & (traj.s[:-1] <= hi) & ok
            cand = _pick(np.where(in_win)[0])
            if cand >= 0 and dist2[cand] <= r2:
                seg, stage = cand, st
                break
    if seg < 0:
        cand = _pick(np.where(ok)[0])
        if cand == -2:
            # cand == -2 : a genuine CROSS-LANE near-tie with no s_prev to
            # break it.  (Same-lane micro-segment ties were already collapsed
            # inside _pick, so this only fires when two DIFFERENT lanes are
            # within tie_eps of the pose -- e.g. equidistant antiparallel
            # lanes.)  Ambiguous: refuse and let the caller stop (A2 item 7).
            # NOTE: the earlier R5 no-s_prev argmin fallback applied to ALL
            # near-ties; the lane collapse made it unnecessary for same-lane
            # micro-segments, so it is revoked here -- an arbitrary pick
            # between two lanes would be a silent coin flip.
            return -1, 0.0, float("nan"), float(s_prev or 0.0), 3
        if cand < 0:
            # cand == -1 : the heading gate rejected EVERY segment.
            # No unconstrained fallback: the gate is binding (fail-open
            # closed).  Refuse, and let the caller stop pre-QP.
            return -1, 0.0, float("nan"), float(s_prev or 0.0), 3
        seg, stage = cand, 2
    nx_, ny_ = -ty[seg], tx[seg]
    lat = (px - proj_x[seg]) * nx_ + (py - proj_y[seg]) * ny_
    arc = traj.s[seg] + along_c[seg]
    w = float(along_c[seg] / lengths[seg]) if lengths[seg] > 1e-12 else 0.0
    return seg, min(max(w, 0.0), 1.0), float(lat), float(arc), stage

def frenet_state(
    traj: Trajectory,
    state: KinematicState,
    lookahead_m: float = 0.0,
) -> Tuple[TrackPoint, np.ndarray]:
    """Error state relative to the reference.

    * projects the pose onto the path,
    * optionally shifts the anchor forward by ``lookahead_m`` along the path
      (compensates discrete-time delay; documented in the derivation doc),
    * returns ``(anchor, err)`` with ``err = [e_y, e_psi, v, omega]`` where
      the v/omega entries are the *absolute* robot velocities (the reference
      velocity/curvature enter the MPC through the reference state, not here).

    Deterministic behaviour on the ends: clamping, never wraps around.
    """
    _, _, e_y, arc, _ = closest_point(traj, state.x, state.y)
    if lookahead_m > 0.0:
        arc = min(arc + lookahead_m, traj.s[-1])
    anchor = traj.sample_by_s(arc)
    e_psi = wrap_angle(state.yaw - anchor.yaw)
    err = np.array([e_y, e_psi, state.v, state.omega], dtype=float)
    return anchor, err


def frenet_state_staged(
    traj: Trajectory,
    state: KinematicState,
    lookahead_m: float = 0.0,
    yaw: Optional[float] = None,
    s_prev: Optional[float] = None,
) -> Tuple[TrackPoint, np.ndarray, int, float]:
    """A3.2 controller projection front: raw projection + stage.

    Returns ``(anchor, err, stage, arc)`` where ``arc`` is the RAW
    projection arc (never lookahead-shifted -- that is the value the caller
    feeds back as ``s_prev`` when the cycle is accepted) and ``stage`` is
    0/1/2/3 per A2.  ``stage == 3`` means no trustworthy projection exists;
    the caller must stop BEFORE building any QP input (A3.2).
    """
    _, _, e_y, arc, stage = closest_point(
        traj, state.x, state.y, yaw=yaw, s_prev=s_prev)
    if stage == 3:
        anchor = traj.sample_by_s(0.0)
        err = np.array([float("nan")] * 4, dtype=float)
        return anchor, err, stage, float(s_prev or 0.0)
    arc_a = arc
    if lookahead_m > 0.0:
        arc_a = min(arc + lookahead_m, traj.s[-1])
    anchor = traj.sample_by_s(arc_a)
    e_psi = wrap_angle(state.yaw - anchor.yaw)
    err = np.array([e_y, e_psi, state.v, state.omega], dtype=float)
    return anchor, err, stage, float(arc)



def frenet_error_at_arc(
    traj: Trajectory,
    state: KinematicState,
    arc: float,
    lookahead_m: float = 0.0,
) -> Tuple[TrackPoint, np.ndarray]:
    """A3.1: anchor + error state taken at an ACCEPTED arc (never raw).

    e_y is recomputed in the tangent frame of ``arc``; a rejected cycle must
    not feed the QP an error measured against the rejected candidate's
    segment.  Returns ``(anchor, err)`` with ``err = [e_y, e_psi, v, omega]``.
    """
    a = min(arc + lookahead_m, traj.s[-1]) if lookahead_m > 0.0 else float(arc)
    a = min(max(a, 0.0), traj.s[-1])
    anchor = traj.sample_by_s(a)
    ny_, nx_ = -math.sin(anchor.yaw), math.cos(anchor.yaw)
    # left-normal frame at the accepted arc: e_y = rel . (-sin, cos)
    e_y = (state.x - anchor.x) * (-math.sin(anchor.yaw)) + (state.y - anchor.y) * math.cos(anchor.yaw)
    e_psi = wrap_angle(state.yaw - anchor.yaw)
    err = np.array([e_y, e_psi, state.v, state.omega], dtype=float)
    return anchor, err



def predicted_anchor_by_s(
    traj: Trajectory,
    base_arc: float,
    s_ahead: float,
) -> TrackPoint:
    """Reference anchor at ``base_arc + s_ahead`` (used for the horizon window)."""
    return traj.sample_by_s(min(base_arc + s_ahead, traj.s[-1]))
