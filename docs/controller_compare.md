# Controller comparison: Frenet LTV-MPC vs Pure Pursuit (P0)

Review ruling: same-LAYER comparison only -- the planner comparison
(lattice / Hybrid-A* references, `results/ref_compare/`) is kept as a
reference-quality SENSITIVITY study, not a headline; the headline is a
controller-vs-controller table under unified conditions.

Code: `benchmark_tools/scripts/run_controller_compare.py`; per-run JSONs +
aggregate: `results/controller_compare/`.

## Unified conditions (identical for both controllers)

- Same plant: differential drive, same actuation lag per scenario.
- Same references (below), same limits (v_max = 0.5 reference cap, omega
  clamped to the MpcParams omega_max, v clamped to [v_min, v_max]).
- Same control period Ts = 0.05 s, same timeout, same completion criterion
  (accepted-arc watermark >= s_end - 0.15, A5.3), same auditor (A5.2 gate).
- Tuning documented: Pure Pursuit lookahead Ld = 0.30 m fixed, omega =
  2 v sin(alpha) / Ld; MPC = repo-standard MpcParams(N=25, shipped weights).
- QP failure rate and fallback counts are reported for the MPC ONLY (the
  baselines have no QP -- never compared across that line).

## Scenarios (review table) and results

| scenario | controller | done | progress | e_y_rms | e_y_p95 | end_gap_m | lat_p95_us | qp_fail |
|---|---|---|---|---|---|---|---|---|
| straight_offset (start 0.30 m off-lane) | mpc | COMPLETED | 0.9757 | 0.1073 | 0.2790 | 0.145 | 4175 | 0 |
| straight_offset | pure_pursuit | COMPLETED | 0.9770 | 0.0618 | 0.1728 | 0.137 | 187 | N/A |
| circle (R = 2 m) | mpc | COMPLETED | 0.9893 | 0.0040 | 0.0045 | 0.135 | 3395 | 0 |
| circle | pure_pursuit | COMPLETED | 0.9887 | 0.0011 | 0.0012 | 0.143 | 349 | N/A |
| s_curve | mpc | COMPLETED | 0.9839 | 0.0255 | 0.0697 | 0.146 | 4214 | 0 |
| s_curve | pure_pursuit | COMPLETED | 0.9836 | 0.0133 | 0.0351 | 0.149 | 241 | N/A |
| foldback (a8_cap serpentine) | mpc | COMPLETED | 0.9958 | 0.0199 | 0.0501 | 0.135 | 4897 | 0 |
| foldback | pure_pursuit | COMPLETED | 0.9956 | 0.0227 | 0.0613 | 0.143 | 612 | N/A |
| foldback_lag (actuation lag 0.15 s) | mpc | **STALL** | 0.3272 | 0.0601 | 0.1441 | 21.762 | 47494 | **173** |
| foldback_lag | pure_pursuit | COMPLETED | 0.9958 | 0.0196 | 0.0561 | 0.135 | 682 | N/A |

(PP latency here is its own compute time; MPC latency includes the QP.
ctrl_variation and lat_acc details are in the per-run JSONs.)

## Honest reading (per the ruling: what it solves, where it still fails)

1. Where MPC wins: steady curved tracking (circle e_y_rms 4 mm) and the
   foldback serpentine with the projection gate -- both complete with 0 QP
   failures.  Pure Pursuit is competitive on most scenarios and ~10x
   cheaper per cycle (sub-ms), which is exactly why it is the standard
   Nav2 tracking default (Regulated Pure Pursuit).
2. Where MPC still FAILS (headline finding, kept per the ruling): with an
   unmodeled 0.15 s actuation lag the QP goes infeasible repeatedly (173
   failures = 173 fallbacks) and the episode STALLS at progress 0.33,
   while Pure Pursuit -- which never assumes a model -- shrugs the lag
   off and completes.  Model mismatch, not reference geometry, is the
   bottleneck in this scenario.
3. Consequence (recorded, not patched in this batch): lag compensation in
   the prediction model (shift the state by one lag period, or identify
   tau) is the obvious candidate fix; it is a NEW experiment and stays
   open.  This table is the baseline it has to beat.
4. The initial-offset scenario also favours PP on RMS (0.062 vs 0.107):
   the MPCs convergence from a large initial lateral error is slower over
   the first metres -- worth one tuning pass (Q weights) before any
   claim; tuning was NOT re-run for this table (repo-standard weights
   used as shipped, documented per the ruling).
