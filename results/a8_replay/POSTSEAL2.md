# Post-seal2: planner-side feasibility (2026-09-09)

Ruling implemented on branch postseal2-planner-feasibility-20260909:
the root disease is the PLANNER emitting references the forward-only
predecessor MPC cannot execute; fixes stay planner-side, tracker-side
reverse semantics rejected.  Seal tags untouched; new branch + evidence.

## 1. Guard (trajectory_tools/reference_guard.py)

One gate for every reference entering the tracker:

1. omega bound: max(|v_ref| * |kappa|) <= omega_max  (reuses
   curvature_estimator.omega_violations, the Day 4-5 half);
2. NO sustained reverse: negative-speed arc <= 0.5 m
   (reverse_arc_max_m).  The reverse half is NEW (post-seal2).

Tests: trajectory_tools/tests/test_reference_guard.py (6 cases,
red/green pinned).  Generator self-check: gen_a8_geometries.py emit()
raises when a generated geometry fails the gate -- nothing infeasible
leaves the planner.

## 2. Red/green evidence (guard on the geometries)

OLD a8_backward (reverse_link, from git 77fe26c):

    omega_violations = 0          (omega half CLEAN)
    reverse_arc_m   = 1.719 m     (> 0.5 m threshold)
    feasible        = False
    reason: sustained reverse 1.719 m of negative-speed arc > 0.500 m
            (predecessor MPC is forward-only)

This is exactly the ruling's prediction: the reference enters the guard
RED on the reverse half while looking omega-clean.

NEW geometries (regenerated): a8_backward (capped), a8_cap, a8_nocap,
a8_cap_real -> ALL feasible (reverse_arc <= 0.05 m, omega_ratio <= 1.0).

Reproducibility proof: regenerating with the same real u9 plan
(~/b6_chain/plan/cleaning_path.json) reproduces a8_cap.json,
a8_nocap.json and a8_cap_real.json BYTE-IDENTICAL to the sealed
versions (git diff empty); the ONLY changed file is a8_backward.json --
so the geometry change is isolated to the retired connector.

## 3. a8_backward: reverse_link retired -> forward half-circle cap

Per ruling option A (cap, keep the numeric chain continuous; tracker
reverse semantics rejected as a separate design problem):

- serpentine() no longer emits the reverse link (back out + reverse
  arc + reverse in, gear = -1).  The backward cell keeps its name for
  matrix traceability but its connector is the same forward fold as
  a8_cap; meta records the retirement.
- Historical STALL (sealed matrix): backward STALL on all four cells,
  progress 0.1407-0.1521, both v_min values, v_end negative.

Post-seal2 matrix rerun (new backward, cap/nocap as repro controls):
see the post-seal2 section appended to MATRIX.md and
postseal2_matrix/ (per-cell JSON + full log).

## 4. What was deliberately NOT done

- No tracker reverse semantics (ruling option B rejected: separate
  design problem, does not preserve the numeric chain).
- No tolerance/audit relaxation anywhere; the sealed numbers and tags
  (cloud-seal-20260908 / postseal-20260908) are untouched.
