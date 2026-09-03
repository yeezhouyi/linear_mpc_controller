"""Fit-model tests on synthetic step responses (U7)."""
import numpy as np

from system_identification.fit_models import (
    fit_lag_from_step,
    simulate_first_order_lag,
)


def _step_input(n=400, ts=0.05, v_high=0.8):
    u = np.zeros(n)
    u[50:] = v_high
    return u


def test_recovers_lag_and_gain():
    Ts = 0.05
    tau_true, gain_true = 0.2, 1.0
    u = _step_input()
    y = simulate_first_order_lag(u, tau_true, gain_true, Ts)
    fit = fit_lag_from_step(u, y, Ts, max_delay=0)
    assert fit["vaf"] > 0.999
    assert abs(fit["tau"] - tau_true) < 0.02
    assert abs(fit["gain"] - gain_true) < 0.02


def test_recovers_delay():
    Ts = 0.05
    tau_true, gain_true, d_true = 0.2, 0.9, 3
    u = _step_input()
    y = simulate_first_order_lag(u, tau_true, gain_true, Ts, d=d_true)
    fit = fit_lag_from_step(u, y, Ts, max_delay=6)
    assert fit["d"] == d_true
    assert fit["vaf"] > 0.999
    assert abs(fit["tau"] - tau_true) < 0.05


def test_validation_split_honest():
    """Fit on the first half, score VAF on the second half (varying input so
    the validation signal has non-zero variance)."""
    Ts = 0.05
    rng = np.random.default_rng(5)
    u = np.clip(0.4 + 0.3 * np.sign(rng.uniform(-1, 1, 700)).cumsum(), 0.0, 1.0)
    y = simulate_first_order_lag(u, 0.25, 0.95, Ts)
    half = 350
    fit = fit_lag_from_step(u[:half], y[:half], Ts, max_delay=2)
    pred = simulate_first_order_lag(u[half:], fit["tau"], fit["gain"], Ts, fit["d"], y0=y[half - 1])
    vaf_val = 1.0 - float(np.sum((y[half:] - pred) ** 2)) / float(
        np.sum((y[half:] - np.mean(y[half:])) ** 2))
    # free-run (recursive) simulation over 350 samples accumulates small ARX
    # mismatch; 0.9 is a strict-enough honest threshold for this check.
    assert vaf_val > 0.9
