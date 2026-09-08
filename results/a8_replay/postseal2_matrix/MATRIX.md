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
| backward | 0.0 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | -0.5 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| backward | -0.5 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |

# Replication rows (global_legacy control + raw-arc audit) -- NOT matrix cells, physically separated per A8.3

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.0 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
| nocap | 0.0 | global_legacy | STALL | 4.798 | 4.798 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0042 | 91 | 0.0 |
| backward | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.0 | 0.0 | 0.033 | 0 | 0 | 0.0199 | 0 | 0.313 |
