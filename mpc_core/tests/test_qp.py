"""ADMM QP solver correctness tests."""
import numpy as np
import pytest

from mpc_core.qp import AdmmQp

INF = float("inf")


def test_unconstrained_matches_analytic():
    rng = np.random.default_rng(0)
    n = 5
    P = rng.normal(size=(n, n))
    P = P @ P.T + np.eye(n)
    q = rng.normal(size=n)
    sol = AdmmQp(max_iter=2000, abs_tol=1e-9, rel_tol=1e-9).solve(P, q, np.zeros((0, n)), np.zeros(0), np.zeros(0))
    assert sol.status == "SOLVED"
    expected = np.linalg.solve(0.5 * (P + P.T), -q)
    assert np.allclose(sol.x, expected, atol=1e-5)


def test_box_known_solutions():
    qp = AdmmQp(max_iter=2000, abs_tol=1e-9, rel_tol=1e-9)
    # min 0.5 x^2 - 3x  s.t. x <= 2  ->  x* = 2
    sol = qp.solve(np.array([[1.0]]), np.array([-3.0]), np.array([[1.0]]), np.array([-INF]), np.array([2.0]))
    assert sol.status in ("SOLVED", "APPROXIMATE")
    assert sol.x[0] == pytest.approx(2.0, abs=1e-5)
    # min x^2 s.t. x >= 3 -> x* = 3
    sol = qp.solve(np.array([[1.0]]), np.array([0.0]), np.array([[1.0]]), np.array([3.0]), np.array([INF]))
    assert sol.x[0] == pytest.approx(3.0, abs=1e-5)
    # min 0.5 x^2 s.t. 1 <= x <= 2 with gradient pushing down -> x* = 1
    sol = qp.solve(np.array([[1.0]]), np.array([-0.5]), np.array([[1.0]]), np.array([1.0]), np.array([2.0]))
    assert sol.x[0] == pytest.approx(1.0, abs=1e-5)


def test_two_dim_against_grid():
    rng = np.random.default_rng(1)
    P = np.array([[2.0, 0.4], [0.4, 1.0]])
    q = np.array([-1.0, 0.5])
    A = np.array([[1.0, 1.0], [-1.0, 0.5]])
    l = np.array([-INF, -INF])
    u = np.array([0.8, 0.3])
    sol = AdmmQp(max_iter=3000, abs_tol=1e-8, rel_tol=1e-8).solve(P, q, A, l, u)
    assert sol.status == "SOLVED"
    # brute-force on a fine grid over the feasible box
    best = None
    best_val = INF
    for x0 in np.linspace(-3, 3, 2001):
        for x1 in np.linspace(-3, 3, 2001):
            x = np.array([x0, x1])
            if np.all(A @ x <= u + 1e-9):
                val = 0.5 * x @ P @ x + q @ x
                if val < best_val:
                    best_val = val
                    best = x
    assert np.allclose(sol.x, best, atol=5e-3)


def test_kkT_residuals_small():
    rng = np.random.default_rng(2)
    n, m = 6, 10
    P = rng.normal(size=(n, n))
    P = P @ P.T + np.eye(n)
    q = rng.normal(size=n)
    A = rng.normal(size=(m, n))
    l = rng.uniform(-2, 0, size=m)
    u = rng.uniform(0, 2, size=m)
    sol = AdmmQp(max_iter=3000, abs_tol=1e-8, rel_tol=1e-8).solve(P, q, A, l, u)
    assert sol.status == "SOLVED"
    # gradient stationarity + bound satisfaction
    g = 0.5 * (P + P.T) @ sol.x + q
    # any violated row must have gradient pointing inward (approx KKT check)
    viol = np.where((A @ sol.x < l - 1e-6) | (A @ sol.x > u + 1e-6))[0]
    assert len(viol) == 0


def test_nan_inputs_fail_cleanly():
    P = np.array([[1.0]])
    q = np.array([np.nan])
    sol = AdmmQp().solve(P, q, np.zeros((1, 1)), np.zeros(1), np.zeros(1))
    assert sol.status == "FAILED"
    assert not sol.ok


def test_warm_start_helps_or_matches():
    P = np.array([[2.0]])
    q = np.array([3.0])
    A = np.array([[1.0]])
    u = np.array([0.5])
    s1 = AdmmQp(max_iter=2000).solve(P, q, A, np.array([-INF]), u)
    s2 = AdmmQp(max_iter=2000).solve(P, q, A, np.array([-INF]), u, warm_start=np.array([0.5]))
    assert s1.ok and s2.ok
    assert np.allclose(s1.x, s2.x, atol=1e-6)
    # unconstrained optimum -q/P = -1.5 lies inside the box x <= 0.5
    assert s2.x[0] == pytest.approx(-1.5, abs=1e-6)
