# Final close-out (2026-09-09) — P0 verification, boundaries, reproduction

Companion to `docs/engineering_checklist.md`.  State: branch `main` @
`ae9d21d` (Advanced-round close-out) plus this commit; CI GREEN on both
jobs; tags: `v1.0.0-sealed` on the display head, seal tags
`cloud-seal-20260908` / `postseal-20260908` / `postseal2-20260909`
untouched.

## 1. 309ec22 merge decision

309ec22 (Advanced round: along-path accel/brake profile + production
entry wiring) is ALREADY on `main` (child ae9d21d).  Decision:
**stay merged**.  CI green at ae9d21d; entry regression numbers are in
the 309ec22 message (raw vs entry rows, all certified, v_end=0 on every
entry, no closed-loop degradation).

## 2. Final test verification (run today, main ae9d21d + this commit)

- Python core suites (no ROS): `pytest mpc_core/tests trajectory_tools/tests
  test/test_ros_contract.py` -> all green.
- C++ colcon test -> green (also exercised by CI on every push).
- GitHub Actions both jobs green at ae9d21d and at this commit.

## 3. Reference-feasibility before/after (fixed comparisons)

The before/after is pinned by three artefacts, all reproducible:

| layer | before | after |
|---|---|---|
| reference certification | raw references fed to the tracker (c21d9ff history) | `prepare_tracker_reference` ends with `certify_reference` (real last gate); every served entry passes + v_end=0 (`run_entry_regress`) |
| controller comparison | none | `results/controller_compare/`: MPC vs Pure Pursuit, identical conditions; honest lag-failure row (MPC STALL @0.33, QP 173 fails, PP completes) |
| reference quality (demoted study) | sharp connectors / raw grid chain | `results/ref_compare/`: continuous-curvature caps 4/4, sharp 2/4, raw grid chain 1/4 |

## 4. One reproduction command

```
cd ~/ros2_ws && colcon build --packages-select linear_mpc_controller && \
colcon test --packages-select linear_mpc_controller
python3 -m pytest mpc_core/tests trajectory_tools/tests \
  test/test_ros_contract.py -q
python3 benchmark_tools/scripts/run_controller_compare.py   # controller table
python3 benchmark_tools/scripts/run_entry_regress.py         # entry certification
```

## 5. Boundaries kept public

- No hard-real-time claim: latencies are p95/p99 measurements of the
  controller cycle (`controller_compare`), never a real-time guarantee.
- Model mismatch failure kept on record: 0.15 s unmodeled lag ->
  QP infeasible storm -> STALL at progress 0.33 (`controller_compare`);
  lag compensation is the open candidate fix.
- No claim that a raw recorded 250-point jagged frontier path tracks
  directly: such references fail certification / stall (`a8_cap_real`
  stall case, `ref_compare` raw-grid row).  Production entries run
  through resample -> complete speed/curvature -> accel profile ->
  certify.
- Closed-loop references (circle returning to the start) must be served
  OPEN (endpoint >= 0.5 m from start), else the nearest-point projection
  ties s=0/s=L and the controller starts "complete" (TB3 smoke fix).

## 6. TB3 Gazebo closed-loop smoke (P0 item) — status: FAILED with precise
diagnosis; one bug fixed, one integration bug open

`scripts/tb3_smoke.sh` (+ `tb3_smoke_check.py`) drive headless Gazebo +
TurtleBot3 + trajectory_server + linear_mpc_node + velocity_arbiter.

Verified live: reference served at 2 Hz; MPC node ACTIVE, 20 Hz cycle
publishing `/cmd_vel_mpc`; arbiter forwards to `/cmd_vel`; ros_gz bridge
passes to the gz `/cmd_vel`; `/odom` at ~50 Hz.

Bug fixed here: `trajectory_server._to_path()` now opens closed-loop
tracks (terminal poses trimmed until the endpoint is >= 0.5 m from the
start).  Before the fix the circle's endpoint==start made the projection
tie s=0/s=L and the controller reported REFERENCE_COMPLETE at t=0.

RESOLVED root cause (2026-09-09): the QP failure was NOT a node bug --
it was a build option.  `CMakeLists.txt` declared
`option(LINEAR_MPC_WITH_OSQP ... OFF)`, so plain `colcon build` (and my
earlier incremental builds) compiled the core WITHOUT osqp_solver.cpp
and the node linked the stub QpSolver, whose solve() returns kFailed at
0 iterations / 0 us on every cycle -- health FALLBACK_ACTIVE forever,
robot never moves.  Fix: the option now defaults ON (stub only for
explicit OSQP-free python-core builds).  The osqp_solver.cpp failure
path also gained a one-time diagnostic print.

Verification after the fix (headless Gazebo + TurtleBot3 + circle):
diagnostics health=OK, qp_status=SOLVED (150 iter, ~1.5 ms), fallback
0; `scripts/tb3_smoke.sh` -> **PASS**: driven 20.7 m (~1.6 laps) over
90 s at mean 0.23 m/s, distance-to-reference median 8.6 mm / p95
34 mm / in-band 1.0.  External claim: TB3 closed-loop smoke PASSED.
