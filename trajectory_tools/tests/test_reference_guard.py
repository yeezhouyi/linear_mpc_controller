"""Reference feasibility guard tests (post-seal2).

Red side pins the a8_backward failure mode: a reference that demands a
sustained backward stretch (reverse_link) is infeasible even when its
omega profile is clean -- the tracker must never be asked to absorb it.
Green side pins the capped replacement and the omega bound together.
"""
import numpy as np
import pytest

from trajectory_tools.reference_guard import (
    reference_violations,
    reverse_arc_m,
)


def _serpentine(reverse_first_connector: bool, n_rows: int = 3,
                row_len: float = 5.0, lane_w: float = 0.3,
                ds: float = 0.05):
    """Dense boustrophedon; first connector either a forward half-circle
    cap or the old reverse_link (back out + reverse arc + reverse in)."""
    xs, ys, gs = [], [], []
    cap_r = 0.15

    def push(pts, g):
        for p in pts:
            if xs and np.hypot(p[0] - xs[-1], p[1] - ys[-1]) < 1e-9:
                if gs:
                    gs[-1] = g    # degenerate: update outgoing gear instead
                continue
            xs.append(p[0])
            ys.append(p[1])
            gs.append(g)

    def row(x0, x1, y):
        n = max(2, int(round(abs(x1 - x0) / ds)) + 1)
        return [(x0 + (x1 - x0) * i / (n - 1), y) for i in range(n)]

    x0, x1 = 0.0, row_len
    y = 0.0
    push(row(x0, x1, y), +1.0)
    edge = x1
    for k in range(1, n_rows):
        y_next = y + lane_w
        if k == 1 and reverse_first_connector:
            x_back = edge - 4.0 * cap_r
            push(row(edge, x_back, y), -1.0)          # back out (reverse)
            n_cap = 10
            cap = [(x_back + cap_r * np.cos(-np.pi / 2 + np.pi * i / n_cap),
                    y + lane_w / 2 + cap_r * np.sin(-np.pi / 2 + np.pi * i / n_cap))
                   for i in range(n_cap + 1)]
            cap = [(2 * x_back - px, py) for (px, py) in cap]
            push(cap, -1.0)                           # reverse arc
            push(row(x_back, edge, y_next), -1.0)     # reverse in
        else:
            yc = y + lane_w / 2
            n_cap = max(4, int(round((np.pi * cap_r) / ds)))
            push([(edge + cap_r * np.cos(-np.pi / 2 + np.pi * i / n_cap),
                   yc + cap_r * np.sin(-np.pi / 2 + np.pi * i / n_cap))
                  for i in range(n_cap + 1)], +1.0)
        y = y_next
        new_edge = x0 if edge == x1 else x1
        push(row(edge, new_edge, y), +1.0)
        edge = new_edge

    x = np.asarray(xs)
    y_arr = np.asarray(ys)
    n = x.size
    s = np.zeros(n)
    for i in range(1, n):
        s[i] = s[i - 1] + np.hypot(x[i] - x[i - 1], y_arr[i] - y_arr[i - 1])
    yaw = np.zeros(n)
    for i in range(n):
        j0, j1 = max(0, i - 1), min(n - 1, i + 1)
        yaw[i] = np.arctan2(y_arr[j1] - y_arr[j0], x[j1] - x[j0])
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        arc = 0.5 * ((s[i] - s[i - 1]) + (s[i + 1] - s[i]))
        if arc > 1e-9:
            kappa[i] = ((yaw[i + 1] - yaw[i - 1] + np.pi)
                        % (2 * np.pi) - np.pi) / (2 * arc)
    gear = np.asarray(gs[:-1], dtype=float)
    v = np.array([min(0.5, 2.0 / max(abs(k), 1e-6)) for k in kappa])
    legal = np.abs(kappa) <= 2.0 / 0.15
    v = np.where(legal, np.maximum(v, 0.15), v)
    sign = np.array([1.0 if (gear[i] if i < gear.size else gear[-1]) >= 0
                     else -1.0 for i in range(n)])
    return kappa, v * sign, s


def test_forward_cap_serpentine_is_feasible():
    kappa, v, s = _serpentine(reverse_first_connector=False)
    verdict = reference_violations(kappa, v, 2.0, s=s)
    assert verdict["feasible"], verdict["reasons"]
    assert verdict["omega_violation_count"] == 0
    assert verdict["reverse_arc_m"] == pytest.approx(0.0, abs=1e-9)


def test_reverse_link_serpentine_is_infeasible():
    """The old a8_backward shape: omega-clean but sustained reverse."""
    kappa, v, s = _serpentine(reverse_first_connector=True)
    verdict = reference_violations(kappa, v, 2.0, s=s)
    assert not verdict["feasible"]
    assert verdict["omega_violation_count"] == 0, \
        "failure mode is the reverse half, not the omega half"
    assert verdict["reverse_arc_m"] > 1.0
    assert any("reverse" in r for r in verdict["reasons"])


def test_omega_violation_still_flagged():
    kappa = np.array([0.0, 10.0, 10.0, 0.0])
    v = np.array([0.5, 0.5, 0.5, 0.5])   # 0.5 * 10 = 5.0 rad/s > 2.0
    verdict = reference_violations(kappa, v, 2.0, s=np.arange(4.0))
    assert not verdict["feasible"]
    assert verdict["omega_violation_count"] == 2
    assert verdict["omega_max_ratio"] == pytest.approx(2.5)


def test_brief_reverse_nudge_below_threshold_is_allowed():
    """A momentary sign flip (one short sample) is not a sustained
    reverse; the gate must not fire on sub-threshold arcs."""
    s = np.array([0.0, 0.1, 0.2, 0.3])
    v = np.array([0.3, -0.1, 0.3, 0.3])
    kappa = np.zeros(4)
    verdict = reference_violations(kappa, v, 2.0, s=s)
    assert verdict["feasible"], verdict["reasons"]
    assert reverse_arc_m(v, s) == pytest.approx(0.1)


def test_boundary_arc_exactly_at_threshold_is_allowed():
    s = np.array([0.0, 0.25, 0.5, 0.6])
    v = np.array([-0.3, -0.3, 0.3, 0.3])   # exactly 0.5 m reverse
    verdict = reference_violations(np.zeros(4), v, 2.0, s=s)
    assert verdict["feasible"], verdict["reasons"]


def test_empty_input_is_feasible():
    verdict = reference_violations(np.array([]), np.array([]))
    assert verdict["feasible"]
    assert verdict["reverse_arc_m"] == 0.0
