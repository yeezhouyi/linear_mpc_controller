#!/usr/bin/env python3
"""Collect a step response from the reference plant and fit lag+delay.

Offline demonstration of the C5 flow (Gazebo data collection uses the same
fit functions via scripts/fit_delay_model.py on rosbag/CSV data).

Usage:
    python system_identification/scripts/collect_step_response.py --out sysid_demo.csv
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from mpc_core.model import DifferentialDrivePlant  # noqa: E402
from system_identification.fit_models import fit_lag_from_step, simulate_first_order_lag  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="outputs/sysid_step_response.csv")
    ap.add_argument("--Ts", type=float, default=0.05)
    ap.add_argument("--lag", type=float, default=0.12)
    ap.add_argument("--delay-steps", type=int, default=2)
    args = ap.parse_args()

    Ts = args.Ts
    n = 300
    u = np.zeros(n)
    u[40:] = 0.6  # velocity step 0 -> 0.6 m/s
    # plant with lag; emulate integer command delay by shifting application
    plant = DifferentialDrivePlant(v0=0.0, lag_s=args.lag)
    v_cmd_hist = []
    y = []
    for k in range(n):
        cmd = u[k - args.delay_steps] if k >= args.delay_steps else 0.0
        plant.step(cmd, 0.0, Ts)
        y.append(plant.state.v)
        v_cmd_hist.append(cmd)
    y = np.array(y)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["k", "u_cmd", "v"])
        for k in range(n):
            w.writerow([k, u[k], round(float(y[k]), 6)])
    print(f"wrote {args.out}")

    # fit (excluding the pre-step region)
    fit = fit_lag_from_step(u, y, Ts, max_delay=6)
    print(f"fit: d={fit['d']} tau={fit['tau']:.3f} gain={fit['gain']:.3f} vaf={fit['vaf']:.3f} "
          f"(true lag {args.lag}, delay {args.delay_steps})")


if __name__ == "__main__":
    main()
