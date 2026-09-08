"""Planner-side reference feasibility guard (post-seal2 ruling).

One gate for EVERY reference handed to the tracker.  A reference is
feasible only when BOTH halves hold:

1. Omega bound (Day 4-5, reused from curvature_estimator.omega_violations):
   max(|v_ref| * |kappa|) <= omega_max.

2. No sustained reverse: the total arc length whose reference speed is
   negative must stay below reverse_arc_max_m.  The predecessor MPC has
   no reverse semantics -- a reference that demands a sustained backward
   stretch (e.g. a reverse_link connector: back out of the row, reverse
   around a half circle, reverse in) stalls the tracker at the first
   v_min (a8_backward: STALL at progress 0.14-0.15 on all four matrix
   cells).  Such references are a planner bug and must be fixed at the
   planner (replace the reverse link with a forward semicircular cap),
   never absorbed by the tracker.
"""
from __future__ import annotations

import numpy as np

from trajectory_tools.curvature_estimator import omega_violations

DEFAULT_OMEGA_MAX = 2.0
DEFAULT_REVERSE_ARC_MAX_M = 0.5


def reverse_arc_m(v: np.ndarray, s: np.ndarray | None = None) -> float:
    """Total arc length driven with negative reference speed.

    s is the per-point arc coordinate; when omitted every sample counts
    as one metre (documented degenerate unit -- always pass s when the
    trajectory carries one).
    """
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return 0.0
    rev = v < 0.0
    if not rev.any():
        return 0.0
    if s is None:
        return float(rev.sum())
    s = np.asarray(s, dtype=float)
    # Per-sample arc lengths; s may be longer than v by one (outgoing-gear
    # convention repeats the last point), so truncate to the shorter array.
    n = min(v.size, s.size)
    ds = np.zeros(n)
    ds[1:] = np.diff(s[:n])
    return float(ds[rev[:n]].sum())


def reference_violations(
    kappa: np.ndarray,
    v: np.ndarray,
    omega_max: float = DEFAULT_OMEGA_MAX,
    s: np.ndarray | None = None,
    reverse_arc_max_m: float = DEFAULT_REVERSE_ARC_MAX_M,
) -> dict:
    """Full feasibility verdict for a candidate reference.

    Returns a dict with the omega half (violation indices + max ratio),
    the reverse half (sustained backward arc in metres) and an overall
    feasible flag with human-readable reasons (empty when clean).
    """
    idx, ratio = omega_violations(kappa, v, omega_max)
    rev_m = reverse_arc_m(v, s)
    reasons = []
    if idx.size:
        reasons.append(
            "omega bound: %d point(s) with |v|*|kappa| > %.3g "
            "(max ratio %.3f)" % (idx.size, omega_max, ratio))
    if rev_m > reverse_arc_max_m:
        reasons.append(
            "sustained reverse: %.3f m of negative-speed arc > %.3f m "
            "(predecessor MPC is forward-only)" % (rev_m, reverse_arc_max_m))
    return {
        "omega_violation_count": int(idx.size),
        "omega_violation_indices": idx.tolist(),
        "omega_max_ratio": ratio,
        "reverse_arc_m": rev_m,
        "reverse_arc_max_m": float(reverse_arc_max_m),
        "feasible": not reasons,
        "reasons": reasons,
    }
