#!/usr/bin/env python3
"""Generate golden QP vectors for the C++ OsqpSolver cross-check (A1).

Builds the same condensed MPC QP as test_qp_cycle.cpp via the Python
reference implementation (LinearMpcController internal builder), solves it
with the in-repo AdmmQp reference solver, and writes the problem + solution
as a whitespace-separated text file:

    n m
    H (n*n, row-major)
    q (n)
    C (m*n, row-major)
    l (m)
    u (m)
    x_golden (n)

The C++ test (test_qp_golden.cpp) re-solves with OsqpSolver and compares.
Regenerate with:
    python3 mpc_core/tools/generate_qp_golden.py > test/golden_qp_vectors.txt
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mpc_core.model import build_ltv_window  # noqa: E402
from mpc_core.mpc import (  # noqa: E402
    LinearMpcController,
    reference_state_vector,
)
from mpc_core.qp import AdmmQp  # noqa: E402
from mpc_core.types import MpcParams  # noqa: E402
from trajectory_tools.reference_trajectory import (  # noqa: E402
    generate_benchmark_tracks,
)


def main() -> None:
    params = MpcParams(N=25)
    traj = generate_benchmark_tracks()["straight"]
    controller = LinearMpcController(params, traj)
    controller.set_reference(traj)

    x0 = np.array([0.05, 0.1, 0.0, 0.0])
    from mpc_core.frenet import frenet_state

    anchor, _ = frenet_state(traj, x0_to_state(x0), params.lookahead_m)
    As, Bs, anchors = build_ltv_window(traj, anchor.s, params.Ts, params.N)
    H, q, C, l, u = controller._build_condensed_qp(x0, As, Bs, anchors)

    solver = AdmmQp(max_iter=20000, abs_tol=1e-8, rel_tol=1e-7)
    res = solver.solve(H, q, C, l, u, warm_start=np.zeros(H.shape[0]))
    if not res.ok:
        raise SystemExit(f"golden generation failed: {res.status}")

    n, m = H.shape[0], C.shape[0]
    out = [f"{n} {m}"]
    for row in H:
        out.append(" ".join("%.17g" % v for v in row))
    out.append(" ".join("%.17g" % v for v in q))
    for row in C:
        out.append(" ".join("%.17g" % v for v in row))
    # +-inf is not reliably parseable by std::istream; use finite sentinels
    l_f = np.where(np.isfinite(l), l, -1e30)
    u_f = np.where(np.isfinite(u), u, 1e30)
    out.append(" ".join("%.17g" % v for v in l_f))
    out.append(" ".join("%.17g" % v for v in u_f))
    out.append(" ".join("%.17g" % v for v in res.x))
    sys.stdout.write("\n".join(out) + "\n")


def x0_to_state(x0):
    """Error-vector x0 -> a plant state that produces it on the straight line."""
    from mpc_core.types import KinematicState

    return KinematicState(x=0.0, y=x0[0], yaw=x0[1], v=x0[2], omega=x0[3])


if __name__ == "__main__":
    main()
