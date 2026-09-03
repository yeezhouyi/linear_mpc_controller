"""Discretisation & linearisation validation (U2)."""
import numpy as np
import pytest

from mpc_core.model import (
    build_ltv_window,
    discretize_euler,
    discretize_zoh,
    expm_dense,
    linear_continuous_matrices,
    nonlinear_error_derivative,
)
from trajectory_tools.reference_trajectory import make_circle, make_straight


def test_expm_basic_properties():
    A = np.array([[0.0, 1.0], [-2.0, -3.0]])
    E = expm_dense(A)
    E_inv = expm_dense(-A)
    assert np.allclose(E @ E_inv, np.eye(2), atol=1e-10)
    # diag case
    D = np.diag([0.5, -1.0, 2.0])
    assert np.allclose(expm_dense(D), np.diag(np.exp([0.5, -1.0, 2.0])), atol=1e-10)
    # expm(0) = I
    assert np.allclose(expm_dense(np.zeros((3, 3))), np.eye(3), atol=1e-12)


def test_linearization_matches_numeric_jacobian():
    rng = np.random.default_rng(7)
    for _ in range(5):
        v_r = rng.uniform(0.3, 1.2)
        kappa = rng.uniform(-0.8, 0.8)
        A_c, B_c = linear_continuous_matrices(v_r, kappa)

        def f(x):
            return nonlinear_error_derivative(x, v_r, kappa)

        x0 = np.array([0.0, 0.0, v_r, kappa * v_r])
        # finite-difference of the nonlinear dynamics at the reference point
        eps = 1e-7
        J_num = np.zeros((4, 4))
        for j in range(4):
            xp = x0.copy()
            xm = x0.copy()
            xp[j] += eps
            xm[j] -= eps
            J_num[:, j] = (f(xp) - f(xm)) / (2 * eps)
        # inputs affect rows 2,3 only
        assert np.allclose(A_c, J_num, atol=1e-4), f"v_r={v_r} kappa={kappa}\nA_c\n{A_c}\nJ_num\n{J_num}"
        B_num = np.zeros((4, 2))
        for j in range(2):
            up = x0.copy()
            up[2 + j] += 1.0  # acceleration applied in the model? not in f
            # B relates u (a, alpha) to xdot: rows 2,3 get +1
            pass
        B_num[2, 0] = 1.0
        B_num[3, 1] = 1.0
        assert np.allclose(B_c, B_num, atol=1e-12)


def test_equilibrium_has_zero_affine_term():
    v_r, kappa = 0.8, 0.5
    x_eq = np.array([0.0, 0.0, v_r, kappa * v_r])
    f = nonlinear_error_derivative(x_eq, v_r, kappa)
    assert np.allclose(f, 0.0, atol=1e-12)


def test_zoh_better_than_euler_on_tight_circle():
    """ZOH must beat first-order Euler for the *linear continuous* model
    x_dot = A_c x + B_c u (isolates the discretisation error)."""
    v_r, kappa, Ts = 0.6, 1.0 / 2.0, 0.05
    A_c, B_c = linear_continuous_matrices(v_r, kappa)
    A_z, B_z = discretize_zoh(A_c, B_c, Ts)
    A_e, B_e = discretize_euler(A_c, B_c, Ts)

    x0 = np.array([0.05, 0.03, v_r - 0.05, kappa * v_r + 0.02])  # small offset
    u = np.array([0.2, 0.1])

    # high-rate truth: fine integration of the *linear* continuous dynamics
    n_sub = 2000
    dt = Ts / n_sub
    x = x0.copy()
    for _ in range(n_sub):
        x = x + (A_c @ x + B_c @ u) * dt
    truth = x

    x_z = A_z @ x0 + B_z @ u
    x_e = A_e @ x0 + B_e @ u
    err_z = np.linalg.norm(x_z - truth)
    err_e = np.linalg.norm(x_e - truth)
    assert err_z < err_e
    assert err_z < 1e-4, f"ZOH too inaccurate: {err_z}"


def test_window_reflects_curved_reference():
    traj = make_circle(radius=2.0, ds=0.01)
    As, Bs, anchors = build_ltv_window(traj, base_arc=0.0, Ts=0.05, N=10)
    assert len(As) == 10 and len(Bs) == 10 and len(anchors) == 11
    # anchors advance at reference speed v=0.6 -> ~0.03 m per step
    s_adv = anchors[1].s - anchors[0].s
    assert s_adv == pytest.approx(0.6 * 0.05, abs=1e-3)
    # kappa constant on the circle
    assert abs(anchors[5].kappa - 0.5) < 1e-9


def test_window_straight_zero_curvature():
    traj = make_straight(length=8.0)
    As, Bs, anchors = build_ltv_window(traj, base_arc=1.0, Ts=0.05, N=5)
    A = As[0]
    # straight-line reference: A[0,1] = v_r, no curvature coupling
    assert A[0, 1] == pytest.approx(0.8 * 0.05, abs=1e-6)  # e_y += Ts*v_r*e_psi
    assert abs(A[1, 0]) < 1e-12
