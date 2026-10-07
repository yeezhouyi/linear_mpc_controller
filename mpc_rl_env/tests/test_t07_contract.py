"""Gate F T07 v1 contract tests for the isolated worktree."""
from __future__ import annotations

import numpy as np
import pytest

from tools.t07_contract_env import ACTION_DELTA_OMEGA_SCALE, OBS_DIM, T07Config, T07FastEnv


def test_schema_one_mpc_call_and_action_mapping():
    calls = []
    env = T07FastEnv(mpc=lambda m: (calls.append(dict(m)) or (0.1, 0.2)))
    obs, _ = env.reset(seed=7)
    nxt, _, done, info = env.step([2.0])
    assert obs.shape == (55,) and nxt.shape == (OBS_DIM,)
    assert not done and info["mpc_calls_this_step"] == 1 and len(calls) == 1
    assert info["action_clipped"] == pytest.approx(1.0)
    assert info["delta_omega"] == pytest.approx(ACTION_DELTA_OMEGA_SCALE)


@pytest.mark.parametrize("delay", [0, 1, 2])
def test_fifo_delay_semantics(delay):
    env = T07FastEnv(T07Config(delay_steps=delay), mpc=lambda _m: (0.1, 0.0))
    env.reset(seed=0); _, _, _, first = env.step([0.0]); _, _, _, second = env.step([0.0])
    assert first["applied_command"][0] == pytest.approx(0.1 if delay == 0 else 0.0)
    assert second["applied_command"][0] == pytest.approx(0.0 if delay == 2 else 0.1)


def test_measurement_truth_and_truth_reward():
    env = T07FastEnv(T07Config(measurement_noise_std=0.1, encoder_bias_v=0.02),
                     mpc=lambda m: (m["v"], 0.0))
    env.reset(seed=2); _, reward, _, info = env.step([0.0])
    assert info["truth_state"] != info["measured_state"]
    assert reward == pytest.approx(-(abs(info["truth_state"]["y"]) +
                                      abs(info["truth_state"]["yaw"])))


def test_reset_clears_history_delay_and_command():
    env = T07FastEnv(T07Config(delay_steps=2), mpc=lambda _m: (0.1, 0.1))
    env.reset(seed=1); env.step([1.0]); obs, info = env.reset(seed=1, path_id="right_arc")
    assert info["path_id"] == "right_arc" and np.allclose(obs[:11], obs[11:22])
    assert np.allclose(env.prev_final_command, 0.0) and env.step_count == 0


def test_nonfinite_mpc_absorbs_without_second_call():
    env = T07FastEnv(mpc=lambda _m: (float("nan"), 0.0))
    env.reset(seed=3); _, reward, done, info = env.step([0.0])
    assert done and reward == -1.0 and info["absorbing"]
    calls = env.mpc_call_count; _, _, done2, info2 = env.step([0.0])
    assert done2 and info2["mpc_calls_this_step"] == 0 and env.mpc_call_count == calls


def test_deterministic_seed_and_action_sequence():
    def rollout():
        env = T07FastEnv(T07Config(measurement_noise_std=0.01),
                         mpc=lambda _m: (0.1, 0.05))
        obs, _ = env.reset(seed=42); out = [obs.copy()]
        for action in ([0.0], [0.5], [-0.25], [1.0]):
            obs, _, _, _ = env.step(action); out.append(obs.copy())
        return np.array(out)
    assert np.allclose(rollout(), rollout())

