#!/usr/bin/env python3
"""Run the dependency-light T07 adapter for a deterministic 10k-step check."""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

# Direct execution puts ``tools/`` on sys.path, not the checkout root.  The
# adapter's real-MPC implementation needs the canonical ``mpc_core`` package.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from tools.t07_training_adapter import T07TrainingAdapter
except ModuleNotFoundError:  # direct execution from the tools directory
    from t07_training_adapter import T07TrainingAdapter


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default="artifacts/handoff_luna/phase_f_t07_isolated_worktree")
    args = ap.parse_args()
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)
    # This is the dependency-light contract/throughput gate.  The training
    # entry point separately uses BenchmarkMpcCallback (real canonical MPC);
    # injecting a deterministic callback here keeps the 10k check focused on
    # adapter semantics and makes its runtime reproducible.
    env = T07TrainingAdapter(
        run_name="gate_f_isolated_10k",
        mpc=lambda _measurement: (0.15, 0.0),
    )
    env.reset(seed=123)
    start = time.perf_counter(); finite = True; samples = []
    for i in range(10_000):
        action = float(np.sin(i * 0.01))
        obs, reward, done, info = env.step([action])
        finite = bool(finite and np.all(np.isfinite(obs)) and np.isfinite(reward))
        if i < 20:
            samples.append({"step": i, "action": action,
                            "delta_omega": info["delta_omega"],
                            "final_v": info["final_command"][0],
                            "final_omega": info["final_command"][1],
                            "applied_v": info["applied_command"][0],
                            "applied_omega": info["applied_command"][1],
                            "truth_x": info["truth_state"]["x"],
                            "truth_y": info["truth_state"]["y"]})
        if done:
            finite = False; break
    elapsed = time.perf_counter() - start
    report = {"steps": i + 1, "requested_steps": 10_000,
              "elapsed_s": elapsed, "steps_per_second": (i + 1) / elapsed,
              "mpc_calls": env.inner.mpc_call_count, "finite": finite,
              "done": bool(done), "observation_dim": env.observation_dim,
              "action_dim": env.action_dim, "baseline": "deterministic_callback",
              "manifest": env.manifest(seed=123)}
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    env.write_manifest(out / "manifest.json", seed=123)
    with (out / "step_sample.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(samples[0])); writer.writeheader(); writer.writerows(samples)
    (out / "run.exit_code").write_text("0\n" if finite and i == 9_999 and env.inner.mpc_call_count == 10_000 else "1\n")
    print(json.dumps({k: report[k] for k in ("steps", "mpc_calls", "finite", "done", "observation_dim", "action_dim")}, indent=2))
    return 0 if finite and i == 9_999 and env.inner.mpc_call_count == 10_000 else 1


if __name__ == "__main__":
    raise SystemExit(main())
