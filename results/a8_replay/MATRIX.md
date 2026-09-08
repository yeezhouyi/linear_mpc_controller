# A8 v_min matrix (accepted-arc audit, 12 cells)

A8.0 alignment (real u9 plan, stall target 7.3 m): done=STALL, accepted_arc_final=0.631, raw_arc_final=0.631

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | -0.5 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | -0.5 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| nocap | 0.0 | stateful_windowed | STALL | 4.798 | 4.798 | 0.1523 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| nocap | 0.0 | global_legacy | STALL | 4.798 | 4.798 | 0.1523 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| nocap | -0.5 | stateful_windowed | COMPLETED | 31.359 | 31.359 | 0.961 | 0.0345 | 0.118 | 10 | 10 | 0.0128 | 0 | 0.319 |
| nocap | -0.5 | global_legacy | COMPLETED | 31.365 | 31.365 | 0.9543 | 0.0414 | 0.163 | 10 | 0 | 0.0158 | 0 | 0.313 |
| backward | 0.0 | stateful_windowed | STALL | 4.625 | 4.625 | 0.1407 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 78 | 0.0 |
| backward | 0.0 | global_legacy | STALL | 4.625 | 4.625 | 0.1407 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 78 | 0.0 |
| backward | -0.5 | stateful_windowed | STALL | 4.993 | 5.0 | 0.1521 | 0.0 | 0.025 | 0 | 3 | 0.0232 | 4 | -0.1 |
| backward | -0.5 | global_legacy | STALL | 5.0 | 5.0 | 0.1521 | 0.0 | 0.025 | 0 | 0 | 0.0752 | 26 | -0.05 |

# Replication rows (global_legacy control + raw-arc audit) -- NOT matrix cells, physically separated per A8.3

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.0 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| nocap | 0.0 | global_legacy | STALL | 4.798 | 4.798 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| backward | 0.0 | global_legacy | STALL | 4.625 | 4.625 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 78 | 0.0 |

# Post-seal2 rerun (2026-09-09, branch postseal2-planner-feasibility-20260909)

reverse_link retired per ruling: a8_backward now carries the forward half-circle cap.  Regeneration reproduced a8_cap / a8_nocap / a8_cap_real byte-identically; only a8_backward.json changed.  Sealed table above is kept as the historical record (backward STALL 0.1407-0.1521 on all four cells).

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | -0.5 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| cap | -0.5 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| nocap | 0.0 | stateful_windowed | STALL | 4.798 | 4.798 | 0.1523 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| nocap | 0.0 | global_legacy | STALL | 4.798 | 4.798 | 0.1523 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| nocap | -0.5 | stateful_windowed | COMPLETED | 31.359 | 31.359 | 0.961 | 0.0345 | 0.118 | 10 | 10 | 0.0128 | 0 | 0.319 |
| nocap | -0.5 | global_legacy | COMPLETED | 31.365 | 31.365 | 0.9543 | 0.0414 | 0.163 | 10 | 0 | 0.0158 | 0 | 0.313 |
| backward | 0.0 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | -0.5 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | -0.5 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |

Result: backward moves STALL -> COMPLETED (progress 0.9958, qp_failures 78/26/4 -> 0) on all four cells; cap and nocap rows reproduce the sealed numbers exactly (repro controls).
