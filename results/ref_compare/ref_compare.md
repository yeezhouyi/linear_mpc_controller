# Reference-quality comparison through the SAME MPC tracker

Same u9 waypoint chain; 4 matrix cells per reference (v_min x
projection mode); medians over cells.

| reference | L (m) | completed/4 | progress | e_y_rms | qp_fail | kappa_rms | peak_kappa | lat_acc_rms | gen_ms |
|---|---|---|---|---|---|---|---|---|---|
| a8_cap | 32.34 | 4/4 | 0.9958 | 0.0199 | 0 | 1.6755 | 6.70 | 0.1894 | -1.0 |
| a8_nocap | 31.50 | 2/4 | 0.5533 | 0.0085 | 46 | 2.4219 | 15.71 | 0.0962 | -1.0 |
| lattice_frenet | 31.51 | 3/4 | 0.9485 | 0.0324 | 0 | 2.7493 | 18.22 | 0.1809 | 2.5 |
| hybrid_astar | 31.93 | 1/4 | 0.3909 | 0.0138 | 46 | 2.6790 | 15.99 | 0.1821 | 389.2 |
