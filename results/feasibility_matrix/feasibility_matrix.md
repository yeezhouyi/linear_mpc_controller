# Feasibility matrix (advanced round, 2026-09-09)

Same plant / limits / Ts=0.05 / completion (A5.3 accepted-arc)
/ auditor (A5.2 gate) as results/controller_compare.  Ld=0.30 m;
MPC MpcParams as shipped (v_min=0, windowed).

| scenario | controller | ref | certified | done | progress | e_y_rms | qp_fail | codes |
|---|---|---|---|---|---|---|---|---|
| straight_offset | mpc | raw | True | COMPLETED | 0.9757 | 0.1073 | 0 | - |
| straight_offset | mpc | guarded | True | COMPLETED | 0.9757 | 0.1073 | 0 | - |
| circle | mpc | raw | True | COMPLETED | 0.9893 | 0.004 | 0 | - |
| circle | mpc | guarded | True | COMPLETED | 0.9882 | 0.004 | 0 | - |
| s_curve | mpc | raw | True | COMPLETED | 0.9839 | 0.0255 | 0 | - |
| s_curve | mpc | guarded | True | COMPLETED | 0.9846 | 0.0258 | 0 | - |
| foldback | mpc | raw | True | COMPLETED | 0.9958 | 0.0199 | 0 | - |
| foldback | mpc | guarded | True | COMPLETED | 0.9958 | 0.0103 | 0 | - |
| foldback_ctrl_axis | mpc | guarded | True | COMPLETED | 0.9958 | 0.0103 | 0 | - |
| foldback_ctrl_axis | pure_pursuit | guarded | True | COMPLETED | 0.9954 | 0.0229 | 0 | - |
| c_sustained_reverse | (none) | counter | False | NOT_RUN_REFUSED | 0.0 | 0.0 | 0 | sustained_reverse |
| c_omega_bound | (none) | counter | False | NOT_RUN_REFUSED | 0.0 | 0.0 | 0 | omega_bound |
| c_inplace_rotation | (none) | counter | False | NOT_RUN_REFUSED | 0.0 | 0.0 | 0 | zero_length_segment,inplace_rotation |
| c_nonfinite | (none) | counter | False | NOT_RUN_REFUSED | 0.0 | 0.0 | 0 | nonfinite |

## Conclusions (honest)

- leg(a): every GUARDED reference certified feasible (0 constraint rows); raw references of the four geometries were already feasible, so the guard is inert there and closed-loop did not degrade: max |delta progress| = 0.0011, max |delta e_y_rms| = 0.0096 across raw/guarded MPC pairs.
- leg(b): on the SAME guarded foldback reference, MPC vs Pure Pursuit give the controller-axis comparison (this is the same- reference, different-controller leg the round asks for).
- leg(c): infeasible counterexamples (sustained reverse / omega bound / in-place rotation / non-finite) are refused BEFORE a controller is invoked, each with a stable code: inplace_rotation; nonfinite; omega_bound; sustained_reverse; zero_length_segment.
- Attribution rule kept: any progress/quality difference between raw and guarded MPC rows is a REFERENCE-layer effect, never credited to the MPC algorithm (same controller both sides).

