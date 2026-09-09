# Controller comparison: Frenet LTV-MPC vs Pure Pursuit

Same plant / references / limits / Ts / completion criterion /
auditor (A5.2 gate).  Ld = 0.30 m; MPC MpcParams as shipped.

| scenario | controller | done | progress | e_y_rms | e_y_p95 | end_gap_m | ctrl_var | lat_p95_us | lat_p99_us | qp_fail |
|---|---|---|---|---|---|---|---|---|---|---|
| straight_offset | mpc | COMPLETED | 0.9757 | 0.1073 | 0.2790 | 0.145 | 0.0020 | 4175 | 4590 | 0 |
| straight_offset | pure_pursuit | COMPLETED | 0.9770 | 0.0618 | 0.1728 | 0.137 | 0.0054 | 187 | 401 | N/A |
| circle | mpc | COMPLETED | 0.9893 | 0.0040 | 0.0045 | 0.135 | 0.0005 | 3395 | 4013 | 0 |
| circle | pure_pursuit | COMPLETED | 0.9887 | 0.0011 | 0.0012 | 0.143 | 0.0005 | 349 | 469 | N/A |
| s_curve | mpc | COMPLETED | 0.9839 | 0.0255 | 0.0697 | 0.146 | 0.0070 | 4214 | 4542 | 0 |
| s_curve | pure_pursuit | COMPLETED | 0.9836 | 0.0133 | 0.0351 | 0.149 | 0.0067 | 241 | 362 | N/A |
| foldback | mpc | COMPLETED | 0.9958 | 0.0199 | 0.0501 | 0.135 | 0.0141 | 4897 | 5260 | 0 |
| foldback | pure_pursuit | COMPLETED | 0.9956 | 0.0227 | 0.0613 | 0.143 | 0.0208 | 612 | 775 | N/A |
| foldback_lag | mpc | STALL | 0.3272 | 0.0601 | 0.1441 | 21.762 | 0.0095 | 47494 | 48821 | 173 |
| foldback_lag | pure_pursuit | COMPLETED | 0.9958 | 0.0196 | 0.0561 | 0.135 | 0.0231 | 682 | 897 | N/A |
