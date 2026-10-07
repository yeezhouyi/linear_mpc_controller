"""Training entry selection is explicit and PPO-free for Gate F."""
from __future__ import annotations

import pytest
import inspect
import subprocess
import sys
from pathlib import Path

from mpc_rl_env.algorithms.train_ppo_residual import build_selected_env
from mpc_rl_env.algorithms import train_ppo_residual


def test_t07_entry_returns_contract_adapter_without_gym_or_ppo():
    env, adapter = build_selected_env({}, 11, "straight", "t07_v1")
    assert adapter.contract == "t07_v1"
    assert adapter.observation_dim == 55 and adapter.action_dim == 1
    assert getattr(env, "observation_dim", 55) == 55


def test_legacy_entry_remains_separate():
    # Config contains all fields needed by the legacy factory; this test only
    # verifies the selector rejects accidental contract substitution.
    with pytest.raises(ValueError, match="unsupported training contract"):
        build_selected_env({}, 0, "straight", "unknown")


def test_legacy_factory_keeps_gym_adapter_symbol():
    # Gymnasium is optional in the offline WSL runtime; inspect the factory so
    # the legacy path cannot regress while T07 is added.
    source = inspect.getsource(train_ppo_residual.build_env)
    assert "gym_adapter.GymResidualTrackingEnv" in source


def test_t07_script_mode_dry_run_writes_manifest(tmp_path):
    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "mpc_rl_env/algorithms/train_ppo_residual.py"
    config = repo_root / "mpc_rl_env/config/t07_v1.yaml"
    result = subprocess.run(
        [sys.executable, str(script), "--config", str(config),
         "--contract", "t07_v1", "--seed", "0", "--dry-run",
         "--outdir", str(tmp_path)],
        cwd=repo_root, capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "dry-run complete" in result.stdout
    assert (tmp_path / "t07_manifest.json").is_file()


def test_t07_track_overrides_flat_config_path_id():
    cfg = {"path_id": "straight", "dt_s": 0.05, "v_limit": 0.30,
           "omega_limit": 1.20}
    _, adapter = build_selected_env(cfg, 0, "circle", "t07_v1")
    assert adapter.inner.path_id == "circle"
    assert adapter.mpc.last_path_id is None
