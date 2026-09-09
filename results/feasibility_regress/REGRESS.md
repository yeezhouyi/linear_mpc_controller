# Production-entry regression (必做1 close-out, 2026-09-09)

Same controller (MPC, as shipped), same plant/limits/Ts=0.05, same A5.2/A5.3 audit as run_feasibility_matrix.  The ENTRY row feeds the controller the output of the real production entry
``prepare_tracker_reference``: resample -> complete_speed_curvature -> accel_limited_profile(terminal brake, v_end=0) -> certify.

## Per-scenario rows (raw vs production entry, MPC both sides)

| scenario | ref | certified | discrete speed | done | progress | e_y_rms | qp_fail | steps | v_end_ref |
|---|---|---|---|---|---|---|---|---|---|
| straight_offset | raw | - | - | COMPLETED | 0.9757 | 0.1073 | 0 | 398 | - |
| straight_offset | entry | yes | yes | COMPLETED | 0.9748 | 0.1071 | 0 | 399 | 0.0000 |
| circle | raw | - | - | COMPLETED | 0.9893 | 0.004 | 0 | 839 | - |
| circle | entry | yes | yes | COMPLETED | 0.9888 | 0.004 | 0 | 840 | 0.0000 |
| s_curve | raw | - | - | COMPLETED | 0.9839 | 0.0255 | 0 | 607 | - |
| s_curve | entry | yes | yes | COMPLETED | 0.9839 | 0.0258 | 0 | 608 | 0.0000 |
| foldback | raw | - | - | COMPLETED | 0.9958 | 0.0199 | 0 | 1440 | - |
| foldback | entry | yes | yes | COMPLETED | 0.9956 | 0.0103 | 0 | 2231 | 0.0000 |

## Conclusions (honest)

- Every production-entry reference certifies feasible AND passes the discrete along-path checks (accel_limited_profile output:
 forward ramp + terminal brake, v_end=0 at the last sample).
- The entry genuinely adds terminal deceleration that no earlier reference carried: last-sample v=0.0000 on all four geometries.
- Closed loop does not degrade: max |delta progress| = 0.0009, max |delta e_y_rms| = 0.0096, max delta qp_failures = 0 across raw/entry MPC pairs.
- Cost of the terminal brake is visible and reported: foldback takes 791 extra steps (1440 -> 2231) as the robot slows to the accepted-arc watermark; the three open geometries add <= 1 step.  progress/e_y do not suffer -- foldback e_y_rms even improves (entry 0.0103 vs raw 0.0199) because the reference no longer demands a 0.3 m/s finish into the fold.
- Attribution kept: raw and entry rows share the SAME controller, so any change is a reference-layer effect, never credited to the
 MPC algorithm.

