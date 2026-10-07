"""First interleaved MPC input block contract."""

import numpy as np
import pytest

from mpc_core.mpc import first_control_acceleration


def test_first_control_acceleration_uses_first_interleaved_block():
    values = np.array([0.11, 0.22, 0.33, 0.44, 0.55, 0.66])
    a0, alpha0 = first_control_acceleration(values)
    assert a0 == pytest.approx(0.11)
    assert alpha0 == pytest.approx(0.22)


def test_first_control_acceleration_requires_one_block():
    with pytest.raises(ValueError, match="one control block"):
        first_control_acceleration(np.array([0.11]))
