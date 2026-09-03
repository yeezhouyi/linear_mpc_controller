# Reference-core benchmark (pure linear MPC, offline)

- commit: `25b0929`  config hash: `168144566ded893a`  runs/track: 1

| track | done | steps | e_y_rms | e_y_p95 | e_y_max | e_psi_rms | e_v_rms | qp_mean(us) | qp_p95(us) | qp_fail | fallback | viol |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| straight | COMPLETED | 194 | 0.122 | 0.349 | 0.355 | 0.107 | 0.130 | 1567 | 3026 | 0 | 0 | 0.0e+00 |
| circle | COMPLETED | 421 | 0.055 | 0.160 | 0.248 | 0.058 | 0.044 | 2756 | 6779 | 0 | 0 | 0.0e+00 |
| s_curve | COMPLETED | 324 | 0.073 | 0.229 | 0.255 | 0.069 | 0.067 | 8222 | 11167 | 0 | 0 | 0.0e+00 |
| u_turn | COMPLETED | 246 | 0.090 | 0.248 | 0.255 | 0.083 | 0.061 | 8306 | 10820 | 0 | 0 | 0.0e+00 |
