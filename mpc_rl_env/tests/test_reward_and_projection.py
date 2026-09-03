"""Golden-vector tests for reward & safety projection (R17/R18)."""
import numpy as np
import pytest

from mpc_rl_env.algorithms.safety_projection import SafetyProjection
from mpc_rl_env.envs.reward import RewardWeights, compute_reward


def test_projection_amplitude_and_rate():
    proj = SafetyProjection(v_max=1.5, omega_max=2.0, dv_max_per_step=0.05, dw_max_per_step=0.1)
    prev = np.array([0.5, 0.0])
    # huge residual: raw clamps to amplitude, then rate-limited vs prev
    out = proj.project(np.array([0.5, 0.0]), np.array([3.0, 3.0]), 1.0, prev)
    assert out[0] == pytest.approx(0.55, abs=1e-12)   # +dv_max_per_step
    assert out[1] == pytest.approx(0.1, abs=1e-12)
    # rate limit dominates when amplitude far away
    out2 = proj.project(np.array([0.0, 0.0]), np.array([0.0, 0.0]), 1.0, np.array([1.4, 1.9]))
    assert out2[0] == pytest.approx(1.35, abs=1e-12)
    assert out2[1] == pytest.approx(1.8, abs=1e-12)


def test_projection_respects_hard_limits_after_shaping():
    proj = SafetyProjection(v_max=1.5, omega_max=2.0, dv_max_per_step=0.05, dw_max_per_step=0.1)
    prev = np.array([1.48, 1.95])
    out = proj.project(np.array([1.48, 1.95]), np.array([0.0, 0.0]), 1.0, prev)
    assert out[0] <= 1.5 and out[1] <= 2.0


def test_reward_components():
    w = RewardWeights(w_e=1.0, lam_psi=0.25, w_v=0.2, w_u=0.05, w_p=1.0,
                      w_c=20.0, w_q=2.0, w_stall=1.0)
    r0 = compute_reward(w, 0.0, 0.0, 0.0, 0.0, 0.1, False, 0.0, False, False, True)
    r_bad = compute_reward(w, 1.0, 0.5, 0.3, 0.1, 0.0, True, 0.05, True, True, False)
    assert r0 > r_bad  # completion + clean beats collision + errors
    # monotonicity on error
    r_small = compute_reward(w, 0.1, 0.0, 0.0, 0.0, 0.1, False, 0.0, False, False, False)
    r_large = compute_reward(w, 0.8, 0.0, 0.0, 0.0, 0.1, False, 0.0, False, False, False)
    assert r_small > r_large
