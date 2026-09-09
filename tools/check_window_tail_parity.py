#!/usr/bin/env python3
"""Cross-language parity for the LTV prediction-window tail (review fix 2).

Builds the SAME path (straight head v=0.4 kappa=0, arc tail v=1.0 kappa=1.0)
in C++ (tools/window_tail_dump) and in python (mpc_core.model.build_ltv_window)
and compares the reference rows used by the QP for three window bases:
mid-path, just-before-the-end, and past-the-end.

Before the fix the C++ tail after the end crossing sampled the path START,
so the near/past rows diverged from python; the regression is caught here.

Usage: python3 tools/check_window_tail_parity.py <path-to-window_tail_dump>
       (the dump path is REQUIRED; a missing binary is an ERROR,
       never a silent pass)
"""
from __future__ import annotations

import math
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mpc_core.model import build_ltv_window, reference_state_vector  # noqa: E402
from mpc_core.types import Trajectory  # noqa: E402

DS = 0.01
L1 = 2.0
R = 1.0
ANGLE = 1.5
TS = 0.05
N = 8
TOL = 1e-6


def make_path() -> Trajectory:
    s, x, y, yaw, kappa, v = [], [], [], [], [], []
    n_st = int(round(L1 / DS))
    for i in range(n_st + 1):
        s.append(i * DS)
        x.append(i * DS)
        y.append(0.0)
        yaw.append(0.0)
        kappa.append(0.0)
        v.append(0.4)
    n_arc = int(round(ANGLE / DS))
    for i in range(1, n_arc + 1):
        th = i * DS / R
        s.append(L1 + i * DS)
        x.append(L1 + R * math.sin(th))
        y.append(R - R * math.cos(th))
        yaw.append(th)
        kappa.append(1.0 / R)
        v.append(1.0)
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=kappa, v=v)


def main(bin_path: pathlib.Path) -> int:
    if not bin_path.is_file():
        print(f"window_tail_parity: FAIL dump binary not found: {bin_path}")
        return 1
    traj = make_path()
    end = traj.s[-1]
    cases = [("mid", 1.0), ("near", end - 0.10), ("past", end + 0.30)]
    out = subprocess.run([str(bin_path)], capture_output=True, text=True, check=True)
    rows = out.stdout.splitlines()

    fails = 0
    for name, base in cases:
        _, _, anchors = build_ltv_window(traj, base, TS, N)
        for k in range(N):
            ref = reference_state_vector(anchors[k + 1])
            want_v = float(ref[2])
            want_om = float(ref[3])
            line = f"{name} {k} "
            got = [ln for ln in rows if ln.startswith(line)]
            if len(got) != 1:
                print(f"FAIL {name} k={k}: dump line missing")
                fails += 1
                continue
            _, _, v_c, om_c = got[0].split()
            dv = abs(float(v_c) - want_v)
            dom = abs(float(om_c) - want_om)
            if dv > TOL or dom > TOL:
                print(
                    f"FAIL {name} k={k}: C++ (v={v_c},om={om_c}) "
                    f"!= python (v={want_v},om={want_om})"
                )
                fails += 1
    if fails:
        print(f"window_tail_parity: {fails} FAILURES")
        return 1
    print("window_tail_parity: OK")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: check_window_tail_parity.py <path-to-window_tail_dump>")
        sys.exit(2)
    sys.exit(main(pathlib.Path(sys.argv[1])))
