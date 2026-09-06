"""Regression: first-order lag must stay bounded for ANY tau > 0.

The explicit-Euler form diverged for tau < Ts/2 (sampled lag range
overlaps that band) — episodes exploded within two steps (seed 100).
"""
import numpy as np
import pytest

from mpc_core.model import DifferentialDrivePlant


@pytest.mark.parametrize("lag_s", [0.004, 0.01, 0.024, 0.03, 0.05])
def test_small_tau_bounded(lag_s):
    plant = DifferentialDrivePlant(x0=0, y0=0, yaw0=0, v0=0, omega0=0,
                                   lag_s=lag_s)
    for _ in range(200):
        plant.step(1.2, 0.5, 0.05)
        assert abs(plant.state.v) <= 1.2 + 1e-9
        assert abs(plant.state.omega) <= 0.5 + 1e-9
        assert np.isfinite(plant.state.x)


def test_zero_tau_tracks_command():
    plant = DifferentialDrivePlant(x0=0, y0=0, yaw0=0, v0=0, omega0=0,
                                   lag_s=0.0)
    plant.step(1.2, 0.5, 0.05)
    assert plant.state.v == pytest.approx(1.2)
    assert plant.state.omega == pytest.approx(0.5)
