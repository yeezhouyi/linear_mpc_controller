"""system_identification: first-order lag + delay fitting (U7/C5).

Fits the discrete ARX model
    ``y[k+1] = a y[k] + b u[k-d]``
(discrete first-order lag with integer delay ``d``) from step-response data,
then recovers continuous parameters ``tau = -Ts / ln(a)`` and
``gain = b / (1 - a)``.  Validation is on an independent data split (VAF).
"""
from __future__ import annotations

import numpy as np


def fit_lag_from_step(u_full: np.ndarray, y_full: np.ndarray, Ts: float,
                      max_delay: int = 5) -> dict:
    """Scan candidate delays d and least-squares fit ``a, b``.

    Returns dict(a, b, d, tau, gain, vaf) of the best (highest VAF) model.
    """
    u_full = np.asarray(u_full, dtype=float)
    y_full = np.asarray(y_full, dtype=float)
    n = len(y_full)
    results = []
    for d in range(0, max_delay + 1):
        k_start = d
        k_end = n - 1
        rows = k_end - k_start
        if rows < 4:
            continue
        A = np.column_stack([
            y_full[k_start:k_end],
            u_full[k_start - d:k_end - d],
        ])
        bvec = y_full[k_start + 1:k_end + 1]
        try:
            coef, *_ = np.linalg.lstsq(A, bvec, rcond=None)
        except np.linalg.LinAlgError:
            continue
        a, b = float(coef[0]), float(coef[1])
        pred = A @ coef
        denom = float(np.sum((bvec - np.mean(bvec)) ** 2))
        vaf = 1.0 - float(np.sum((bvec - pred) ** 2)) / max(denom, 1e-12)
        if 0.0 < a < 1.0:
            tau = -Ts / np.log(a)
        else:
            tau = float("inf")
        gain = b / max(1.0 - a, 1e-12)
        results.append({"a": a, "b": b, "d": d, "tau": tau, "gain": gain, "vaf": vaf})
    if not results:
        return {}
    results.sort(key=lambda r: -r["vaf"])
    return results[0]


def simulate_first_order_lag(u: np.ndarray, tau: float, gain: float, Ts: float,
                             d: int = 0, y0: float = 0.0) -> np.ndarray:
    """Simulate y[k+1] = a y[k] + b u[k-d] from continuous tau/gain."""
    a = np.exp(-Ts / tau)
    b = gain * (1.0 - a)
    n = len(u)
    y = np.zeros(n)
    y[0] = y0
    for k in range(n - 1):
        uk = u[k - d] if k - d >= 0 else 0.0
        y[k + 1] = a * y[k] + b * uk
    return y
