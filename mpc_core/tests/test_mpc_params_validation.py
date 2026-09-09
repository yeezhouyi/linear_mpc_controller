"""MpcParams contract tests (engineering checklist item 3).

Invalid configurations must be REJECTED at construction with a clear
error -- never silently accepted and discovered mid-mission.
"""
import pytest

from mpc_core.types import MpcParams


def test_default_params_valid():
    p = MpcParams()
    assert p.Ts > 0 and p.N >= 1


@pytest.mark.parametrize("kwargs,fragment", [
    (dict(Ts=0.0), "Ts"),
    (dict(N=0), "N"),
    (dict(v_min=1.0, v_max=0.5), "v_min"),
    (dict(omega_max=0.0), "omega_max"),
    (dict(a_max=-1.0), "a_max"),
    (dict(qp_max_iter=0), "qp_max_iter"),
    (dict(controller_projection_mode="bogus"), "controller_projection_mode"),
])
def test_invalid_params_rejected(kwargs, fragment):
    with pytest.raises(ValueError, match=fragment):
        MpcParams(**kwargs)


def test_negative_weight_rejected():
    with pytest.raises(ValueError, match="Q_diag"):
        MpcParams(Q_diag=(-1.0, 10.0, 2.0, 1.0))


def test_negative_v_min_allowed():
    """Reverse v_min is legal (A8 axis) -- must NOT be rejected."""
    p = MpcParams(v_min=-0.5)
    assert p.v_min == -0.5
