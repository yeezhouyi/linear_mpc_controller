"""Planner-side reference feasibility guard (post-seal2 ruling + advanced round).

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

Advanced round (2026-09-09) adds the input-hygiene layer and a single
certifier so the round's acceptance ("an infeasible reference is
IDENTIFIED and a processed reference satisfies the discrete constraint
check") has one stable API:

  * validate_input_path(x, y[, yaw])  -- raw waypoint hygiene: non-finite
    samples, zero-length (duplicate) segments, and in-place rotation (a
    chord of ~0 m that still flips heading; only detectable when yaw is
    given -- a tracker fed such a segment would try to turn without
    moving).

  * certify_reference(x, y, kappa, v, s[, yaw]) -- the single gate for a
    tracker-bound reference: merges the path-hygiene verdict with the
    motion-limit verdict (omega bound + sustained reverse + non-finite
    samples in kappa/v/s).

Every reported reason carries a stable machine code (see ``codes``):
nonfinite / zero_length_segment / inplace_rotation / omega_bound /
sustained_reverse.  The certifier only REPORTS -- a reference it rejects
must be fixed at the planner (re-sample, re-speed, replace reverse
connectors with forward caps) or re-split at the task layer; it is never
silently edited here, and never fed to the controller.
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
    """Full motion-limit feasibility verdict for a candidate reference.

    Covers the omega half (violation indices + max ratio), the reverse
    half (sustained backward arc in metres) and -- advanced round --
    non-finite samples in kappa/v/s (a NaN/inf speed or curvature must
    never reach a controller; silently skipping it would fake feasibility).

    Returns a dict with an overall ``feasible`` flag, human-readable
    ``reasons`` and a machine-parseable ``codes`` list (stable tokens:
    omega_bound / sustained_reverse / nonfinite).
    """
    kappa = np.asarray(kappa, dtype=float)
    v = np.asarray(v, dtype=float)
    n = min(kappa.size, v.size)
    if n == 0:
        return {
            "omega_violation_count": 0,
            "omega_violation_indices": [],
            "omega_max_ratio": 0.0,
            "reverse_arc_m": 0.0,
            "reverse_arc_max_m": float(reverse_arc_max_m),
            "feasible": True,
            "reasons": [],
            "codes": [],
        }

    # --- non-finite hygiene on the motion arrays -------------------------
    nf_k = (~np.isfinite(kappa[:n])).nonzero()[0]
    nf_v = (~np.isfinite(v[:n])).nonzero()[0]
    nf_s: np.ndarray = np.array([], dtype=int)
    if s is not None:
        s_arr = np.asarray(s, dtype=float)
        ns = min(s_arr.size, n)
        nf_s = (~np.isfinite(s_arr[:ns])).nonzero()[0]

    # --- omega half --------------------------------------------------------
    # Compute the ratio over FINITE samples only (omega_violations' own
    # ratio would be NaN when kappa carries a NaN).
    idx, _ = omega_violations(kappa, v, omega_max)
    with np.errstate(invalid="ignore"):
        omega = np.abs(kappa[:n]) * np.abs(v[:n])
    finite = np.isfinite(omega)
    ratio = float(omega[finite].max() / omega_max) if finite.any() else 0.0

    # --- reverse half -------------------------------------------------------
    rev_m = reverse_arc_m(v, s)
    if not np.isfinite(rev_m):          # non-finite s already reported above
        rev_m = 0.0

    reasons: list[str] = []
    codes: list[str] = []
    if nf_k.size or nf_v.size or nf_s.size:
        codes.append("nonfinite")
        reasons.append(
            "nonfinite: %d kappa, %d v, %d s sample(s) (code=nonfinite)"
            % (nf_k.size, nf_v.size, nf_s.size))
    if idx.size:
        codes.append("omega_bound")
        reasons.append(
            "omega bound: %d point(s) with |v|*|kappa| > %.3g "
            "(max ratio %.3f) (code=omega_bound)"
            % (idx.size, omega_max, ratio))
    if rev_m > reverse_arc_max_m:
        codes.append("sustained_reverse")
        reasons.append(
            "sustained reverse: %.3f m of negative-speed arc > %.3f m "
            "(predecessor MPC is forward-only) (code=sustained_reverse)"
            % (rev_m, reverse_arc_max_m))
    return {
        "omega_violation_count": int(idx.size),
        "omega_violation_indices": idx.tolist(),
        "omega_max_ratio": ratio,
        "reverse_arc_m": rev_m,
        "reverse_arc_max_m": float(reverse_arc_max_m),
        "nonfinite_kappa_indices": nf_k.tolist(),
        "nonfinite_v_indices": nf_v.tolist(),
        "nonfinite_s_indices": nf_s.tolist(),
        "feasible": not reasons,
        "reasons": reasons,
        "codes": codes,
    }


def validate_input_path(
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray | None = None,
    ds_min: float = 1e-6,
    yaw_flip_rad: float = 0.05,
) -> dict:
    """Raw waypoint hygiene check (advanced round).

    Flags, with their sample indices:
      nonfinite           -- NaN/inf in x/y (or yaw when given);
      zero_length_segment -- a chord shorter than ``ds_min`` (duplicate);
      inplace_rotation    -- a zero-length chord that still flips heading
                             by more than ``yaw_flip_rad`` (turn in place).

    A path is ``valid`` only when all three are empty.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = min(x.size, y.size)
    reasons: list[str] = []
    codes: list[str] = []
    nf_idx = (~(np.isfinite(x[:n]) & np.isfinite(y[:n]))).nonzero()[0]
    if yaw is not None:
        yaw_arr = np.asarray(yaw, dtype=float)
        ny = min(yaw_arr.size, n)
        nf_idx = np.union1d(
            nf_idx, (~np.isfinite(yaw_arr[:ny])).nonzero()[0]).astype(int)
    if nf_idx.size:
        codes.append("nonfinite")
        reasons.append(
            "nonfinite: %d waypoint(s) at %s (code=nonfinite)"
            % (nf_idx.size, nf_idx[:8].tolist()))

    zl_idx: np.ndarray = np.array([], dtype=int)
    ip_idx: np.ndarray = np.array([], dtype=int)
    if n > 1:
        chord = np.hypot(np.diff(x[:n]), np.diff(y[:n]))
        zl = chord < ds_min
        if zl.any():
            zl_idx = zl.nonzero()[0] + 1   # report the segment-END sample
            codes.append("zero_length_segment")
            reasons.append(
                "zero_length_segment: %d chord(s) < %.1e near sample %s "
                "(code=zero_length_segment)"
                % (int(zl.sum()), ds_min, zl_idx[:8].tolist()))
            if yaw is not None:
                yaw_arr = np.asarray(yaw, dtype=float)
                nm = min(yaw_arr.size, n)
                dpsi = yaw_arr[1:nm] - yaw_arr[:nm - 1]
                dpsi = np.mod(dpsi + np.pi, 2.0 * np.pi) - np.pi
                seg_idx = zl.nonzero()[0]           # segment base indices
                flip = np.abs(dpsi[seg_idx]) > yaw_flip_rad
                if flip.any():
                    ip_idx = (seg_idx[flip] + 1)
                    codes.append("inplace_rotation")
                    reasons.append(
                        "inplace_rotation: %d zero-length chord(s) flip "
                        "heading by > %.3g rad near sample %s "
                        "(code=inplace_rotation)"
                        % (int(flip.sum()), yaw_flip_rad,
                           ip_idx[:8].tolist()))
    valid = not (nf_idx.size or zl_idx.size or ip_idx.size)
    return {
        "valid": bool(valid),
        "nonfinite_indices": nf_idx.tolist(),
        "zero_length_indices": zl_idx.tolist(),
        "inplace_rotation_indices": ip_idx.tolist(),
        "codes": codes,
        "reasons": reasons,
    }


def certify_reference(
    x: np.ndarray,
    y: np.ndarray,
    kappa: np.ndarray,
    v: np.ndarray,
    s: np.ndarray,
    yaw: np.ndarray | None = None,
    omega_max: float = DEFAULT_OMEGA_MAX,
    reverse_arc_max_m: float = DEFAULT_REVERSE_ARC_MAX_M,
    ds_min: float = 1e-6,
    yaw_flip_rad: float = 0.05,
) -> dict:
    """Single feasibility gate for a tracker-bound reference (advanced round).

    Merges validate_input_path (hygiene) with reference_violations (motion
    limits).  ``feasible`` is the conjunction; ``reasons``/``codes`` are the
    merged report.  Callers use this right before handing the reference to a
    controller: an infeasible verdict means the reference must go back to
    the planner / task layer with the code in hand.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = min(x.size, y.size)
    if n == 0:
        return {
            "feasible": False,
            "reasons": ["empty reference (0 points)"],
            "codes": [],
            "path": {"valid": False},
            "motion": {"feasible": False},
        }
    path = validate_input_path(x, y, yaw=yaw, ds_min=ds_min,
                               yaw_flip_rad=yaw_flip_rad)
    kappa = np.asarray(kappa, dtype=float)
    v = np.asarray(v, dtype=float)
    s_arr = np.asarray(s, dtype=float)
    motion = reference_violations(kappa[:n], v[:n], omega_max,
                                  s=s_arr[:n],
                                  reverse_arc_max_m=reverse_arc_max_m)
    reasons = list(path["reasons"]) + list(motion["reasons"])
    codes = list(path["codes"]) + list(motion["codes"])
    return {
        "feasible": bool(path["valid"] and motion["feasible"]),
        "reasons": reasons,
        "codes": codes,
        "path": {"valid": bool(path["valid"])},
        "motion": {
            "feasible": bool(motion["feasible"]),
            "omega_violation_count": motion["omega_violation_count"],
            "omega_max_ratio": motion["omega_max_ratio"],
            "reverse_arc_m": motion["reverse_arc_m"],
            "reverse_arc_max_m": motion["reverse_arc_max_m"],
        },
    }
