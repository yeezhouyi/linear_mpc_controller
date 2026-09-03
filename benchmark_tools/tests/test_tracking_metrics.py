"""Metrics function tests."""
import numpy as np

from benchmark_tools.compute_tracking_metrics import compute_tracking_metrics
from mpc_core.episode import EpisodeResult


def _mk_res():
    r = EpisodeResult(track_name="t")
    r.e_y = [0.1, -0.1, 0.2, -0.2]
    r.e_psi = [0.01, -0.01, 0.02, -0.02]
    r.v = [0.5, 0.51, 0.49, 0.5]
    r.omega = [0.1] * 4
    r.cmd_v = [0.5, 0.52, 0.5, 0.49]
    r.cmd_w = [0.1] * 4
    r.health = [0] * 4
    r.qp_status = ["SOLVED"] * 4
    r.qp_time_us = [1000, 1200, 900, 1100]
    r.qp_iterations = [40] * 4
    r.constraint_violation = [0.0] * 4
    r.completed = True
    r.done_reason = "COMPLETED"
    r.steps = 4
    return r


def test_rms_p95_max():
    m = compute_tracking_metrics(_mk_res())
    assert m["e_y_rms"] == round(float(np.sqrt(np.mean(np.square([0.1, -0.1, 0.2, -0.2])))), 5)
    assert m["e_y_max"] == round(0.2, 5)
    assert m["qp_time_us_mean"] == 1050.0
    assert m["qp_failures"] == 0


def test_empty_lists_safe():
    m = compute_tracking_metrics(EpisodeResult(track_name="x"))
    assert m["completed"] is False
    assert np.isnan(m["e_y_rms"]) or m["e_y_rms"] == 0
