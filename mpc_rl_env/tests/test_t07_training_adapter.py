"""Offline adapter and manifest checks."""
from __future__ import annotations

import json
import numpy as np
import pytest

from tools.t07_training_adapter import (
    ADAPTER_VERSION,
    BenchmarkMpcCallback,
    CONTRACT,
    T07TrainingAdapter,
    make_training_adapter,
    resolve_track_id,
)


def test_explicit_contract_and_dynamic_spaces():
    adapter = make_training_adapter(contract=CONTRACT)
    assert adapter.observation_space.shape == (55,)
    assert adapter.action_space.shape == (1,)
    assert adapter.observation_space.dtype == np.dtype(np.float32)
    assert adapter.action_space.contains(np.array([0.0], dtype=np.float32))
    assert not adapter.action_space.contains(np.array([0.0, 0.0]))


def test_adapter_preserves_step_and_metadata():
    calls = []
    adapter = T07TrainingAdapter(mpc=lambda m: (calls.append(m) or (0.1, 0.0)))
    obs, reset_info = adapter.reset(seed=4, path_id="left_arc")
    nxt, _, done, info = adapter.step([1.0])
    assert obs.shape == (55,) and nxt.shape == (55,) and obs.dtype == np.float32
    assert reset_info["contract"] == CONTRACT and info["adapter_version"] == ADAPTER_VERSION
    assert info["delta_omega"] == pytest.approx(0.05) and not done and len(calls) == 1


def test_manifest_stable_and_writeable(tmp_path):
    adapter = T07TrainingAdapter(run_name="gate_f_review")
    manifest = adapter.manifest(seed=123)
    assert manifest["contract"] == CONTRACT and manifest["observation"]["shape"] == [55]
    assert manifest["action"]["shape"] == [1] and manifest["canonical_checkout"] == "read_only"
    assert manifest["baseline"] == "LinearMpcController"
    assert "circle" in manifest["benchmark_tracks"]
    path = adapter.write_manifest(tmp_path / "manifest.json", seed=123)
    assert json.loads(path.read_text()) == manifest


def test_legacy_contract_is_not_silently_replaced():
    with pytest.raises(ValueError, match="legacy training remains outside"):
        make_training_adapter(contract="legacy")


def test_default_baseline_calls_linear_mpc_and_preserves_track_name():
    adapter = T07TrainingAdapter()
    assert isinstance(adapter.mpc, BenchmarkMpcCallback)
    adapter.reset(seed=0, path_id="circle")
    _, _, done, info = adapter.step([0.0])
    assert not done
    assert info["baseline"] == "LinearMpcController"
    assert info["path_id"] == "circle"
    assert adapter.mpc.calls == 1 and adapter.mpc.last_path_id == "circle"
    # Circle and straight are distinct benchmark trajectories; no fallback
    # to the straight controller is allowed for a circle run.
    assert resolve_track_id("circle") == "circle"
    assert resolve_track_id("straight") == "straight"


def test_unknown_track_fails_closed():
    with pytest.raises(ValueError, match="unsupported T07 path_id"):
        T07TrainingAdapter().reset(path_id="circl")
