#!/usr/bin/env python3
"""Cross-language parity for the kinematic speed guard (Day 4-5).

Runs the C++ adapter completion (tools/adapter_speed_dump, same synthetic
circle) and compares it against the python reference completion
(trajectory_tools.curvature_estimator.complete_speed_curvature).

The dump is invoked with curve_speed == omega_max so the C++ rule
  v = min(v_default, curve_speed/|k|, omega_max/|k|)
reduces to the python rule
  v = min(v_default, omega_max/|k|)
(i.e. the two are compared on the same documented rule).  The test then
asserts the shared invariant max|kappa*v| <= omega_max on BOTH sides.

Skips (exit 0) when the dump binary is not built.
"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "trajectory_tools"))

from curvature_estimator import complete_speed_curvature  # noqa: E402

OMEGA_MAX = 2.0
V_DEFAULT = 0.5
V_MAX = 1.5
R = 0.10
N = 72
TOL = 1e-6


def find_binary() -> Path | None:
    for cand in (
        ROOT / "build_core" / "adapter_speed_dump",
        Path("/home/zhouyi/lmpc_ws/build/linear_mpc_controller/adapter_speed_dump"),
        Path("/home/zhouyi/ros2_ws/build/linear_mpc_controller/adapter_speed_dump"),
    ):
        if cand.exists():
            return cand
    return None


def circle_xy():
    th = np.linspace(0.0, 2.0 * math.pi, N, endpoint=False)
    return R * np.cos(th), R * np.sin(th)


def main() -> int:
    bin_path = find_binary()
    if bin_path is None:
        print("skip: adapter_speed_dump not built")
        return 0

    out = subprocess.run(
        [str(bin_path), "%.6f" % OMEGA_MAX, "%.6f" % OMEGA_MAX],
        capture_output=True, text=True, check=True).stdout.splitlines()
    ratio_cpp = float(out[1].split()[1])
    cpp = [tuple(map(float, ln.split())) for ln in out[3:] if ln.strip()]

    x, y = circle_xy()
    _, kappa_py, v_py = complete_speed_curvature(
        x, y, v_default=V_DEFAULT, v_max=V_MAX, omega_max=OMEGA_MAX)

    assert len(cpp) == len(v_py), "point count mismatch"

    worst_dv = 0.0
    worst_dk = 0.0
    for (k_cpp, v_cpp), k_p, v_p in zip(cpp, kappa_py, v_py):
        worst_dk = max(worst_dk, abs(k_cpp - float(k_p)))
        worst_dv = max(worst_dv, abs(v_cpp - float(v_p)))

    ratio_py = float(np.max(np.abs(kappa_py) * np.abs(v_py)) / OMEGA_MAX)

    ok = True
    if worst_dk > TOL:
        print(f"FAIL kappa mismatch: max|dkappa|={worst_dk:.3e}")
        ok = False
    if worst_dv > TOL:
        print(f"FAIL v mismatch: max|dv|={worst_dv:.3e}")
        ok = False
    if ratio_cpp > 1.0 + 1e-9:
        print(f"FAIL C++ guard: max_omega_ratio={ratio_cpp:.6f} > 1")
        ok = False
    if ratio_py > 1.0 + 1e-9:
        print(f"FAIL python guard: max_omega_ratio={ratio_py:.6f} > 1")
        ok = False

    print(f"points={len(cpp)} max|dkappa|={worst_dk:.3e} max|dv|={worst_dv:.3e} "
          f"ratio_cpp={ratio_cpp:.6f} ratio_py={ratio_py:.6f}")
    print("adapter_speed_parity: PASS" if ok else "adapter_speed_parity: FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
