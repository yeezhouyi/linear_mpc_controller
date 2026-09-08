"""Uniform arc-length resampling tests (Day 4-5).

Resampling is step 1 of the rigid fix order -- the real u9 plan has point
spacing 0.050 ~ 4.550 m (30 gaps > 0.5 m) and finite-difference curvature
on the raw polyline manufactures phantom corners.  These tests pin the
resampler contract: near-uniform spacing, endpoints preserved, total arc
length preserved, degenerate input untouched.
"""
import numpy as np
import pytest

from trajectory_tools.resample import resample_uniform


def test_nonuniform_input_becomes_near_uniform():
    # 5 points with wildly uneven spacing along one axis
    x = np.array([0.0, 0.05, 0.5, 3.0, 5.0])
    y = np.zeros(5)
    xr, yr = resample_uniform(x, y, ds=0.05)
    assert xr[0] == pytest.approx(0.0)
    assert xr[-1] == pytest.approx(5.0)          # endpoints preserved
    seg = np.diff(xr)
    assert np.all(np.abs(seg - 0.05) < 1e-9)     # uniform spacing interior
    total_orig = float(np.abs(np.diff(x)).sum())
    assert xr[-1] - xr[0] == pytest.approx(total_orig)


def test_arc_length_preserved_on_turn():
    # an L path (right angle): raw chord length 2.0, uniform resample keeps
    # total arc within a last-interval remainder
    x = np.array([0.0, 1.0, 1.0])
    y = np.array([0.0, 0.0, 1.0])
    xr, yr = resample_uniform(x, y, ds=0.05)
    dx = np.diff(xr)
    dy = np.diff(yr)
    total = float(np.hypot(dx, dy).sum())
    assert total == pytest.approx(2.0, abs=0.051)   # endpoint exactness
    assert xr.size >= 40


def test_uniform_input_unchanged_shape():
    t = np.linspace(0.0, 1.0, 21)
    x = t
    y = np.zeros_like(t)
    xr, yr = resample_uniform(x, y, ds=0.05)
    assert xr.size == 21
    assert np.allclose(xr, x, atol=1e-12)


def test_degenerate_input_returned_unchanged():
    x = np.array([1.0, 1.0])
    y = np.array([2.0, 2.0])
    xr, yr = resample_uniform(x, y, ds=0.05)       # zero total length
    assert xr.size == 2 and np.array_equal(xr, x)
    xr2, yr2 = resample_uniform(np.array([3.0]), np.array([4.0]), ds=0.05)
    assert xr2.size == 1


def test_ds_argument_scales_count():
    x = np.array([0.0, 5.0])
    y = np.array([0.0, 0.0])
    xr, _ = resample_uniform(x, y, ds=0.5)
    assert xr.size == 11
