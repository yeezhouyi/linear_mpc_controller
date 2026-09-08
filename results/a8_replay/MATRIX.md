# A8 v_min matrix (accepted-arc audit, 12 cells)

A8.0 alignment (real u9 plan, stall target 7.3 m): done=STALL, accepted_arc_final=0.988, raw_arc_final=0.988

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.025 | 0 | 0 | 0.0155 | 0 | 0.314 |
| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.025 | 0 | 0 | 0.0155 | 0 | 0.314 |
| cap | -0.5 | stateful_windowed | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.025 | 0 | 0 | 0.0155 | 0 | 0.314 |
| cap | -0.5 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.9958 | 0.0 | 0.025 | 0 | 0 | 0.0155 | 0 | 0.314 |
| nocap | 0.0 | stateful_windowed | STALL | 4.815 | 4.815 | 0.1529 | 0.0 | 0.025 | 0 | 0 | 0.0054 | 94 | 0.0 |
| nocap | 0.0 | global_legacy | STALL | 4.815 | 4.815 | 0.1529 | 0.0 | 0.025 | 0 | 0 | 0.0054 | 94 | 0.0 |
| nocap | -0.5 | stateful_windowed | COMPLETED | 31.353 | 31.353 | 0.9565 | 0.0389 | 0.143 | 10 | 16 | 0.0187 | 0 | 0.325 |
| nocap | -0.5 | global_legacy | COMPLETED | 31.361 | 31.361 | 0.9515 | 0.0441 | 0.173 | 10 | 0 | 0.0258 | 0 | 0.317 |
| backward | 0.0 | stateful_windowed | STALL | 4.524 | 4.524 | 0.1376 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 105 | 0.0 |
| backward | 0.0 | global_legacy | STALL | 4.524 | 4.524 | 0.1376 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 105 | 0.0 |
| backward | -0.5 | stateful_windowed | STALL | 5.0 | 5.0 | 0.1521 | 0.0 | 0.025 | 0 | 0 | 0.0256 | 164 | 0.0 |
| backward | -0.5 | global_legacy | STALL | 5.0 | 5.0 | 0.1521 | 0.0 | 0.025 | 0 | 0 | 0.0256 | 164 | 0.0 |

# Replication rows (global_legacy control + raw-arc audit) -- NOT matrix cells, physically separated per A8.3

| geometry | v_min | controller_projection_mode | done_reason | raw_arc_final | accepted_arc_final | progress_ratio | skipped_arc_ratio | max_arc_jump | projection_jump_count | projection_reacquire_count | e_y_rms | qp_failures | v_end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

| cap | 0.0 | global_legacy | COMPLETED | 32.209 | 32.209 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0155 | 0 | 0.314 |
| nocap | 0.0 | global_legacy | STALL | 4.815 | 4.815 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0054 | 94 | 0.0 |
| backward | 0.0 | global_legacy | STALL | 4.524 | 4.524 | 0.0 | 0.0 | 0.025 | 0 | 0 | 0.0002 | 105 | 0.0 |
