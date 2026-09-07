"""A7.3 guards: one allowance formula, physical displacement, no copies.
Two layers, mirroring test_projection_callsites.py:
  * SOURCE guards -- every production progress ledger must route its
    reachability decision through ``mpc_core.progress_gate`` and must not
    reintroduce a velocity-derived allowance (``v * cos(e_psi) * Ts``).
  * BEHAVIOUR guards -- the properties the shared gate must hold.  The
    load-bearing one is ``test_wrong_lane_cannot_grant_itself_budget``.
"""
import math
import re
from pathlib import Path

import numpy as np

from mpc_core.progress_gate import (
    ALLOWANCE_CAP_M,
    ODOM_STEP_NOISE_M,
    PROJECTION_MARGIN_M,
    ProgressAllowanceGate,
)
from mpc_core.types import MpcParams, Trajectory

REPO = Path(__file__).resolve().parents[2]
# The three ledgers that each carried their own copy of the gate.
LEDGER_SITES = [
    "mpc_core/episode.py",
    "benchmark_tools/scripts/replay_path_mpc.py",
    "mpc_rl_env/envs/fast_tracking_env.py",
]
# ``<state>.v * math.cos(...)`` -- the shape of the removed formula.
_VELOCITY_ALLOWANCE = re.compile(r"\.v\s*\*\s*math\.cos\s*\(")


# ---------------------------------------------------------------- source
def test_no_velocity_derived_allowance_in_ledgers():
    """No ledger may recompute the allowance from a velocity sample."""
    bad = []
    for rel in LEDGER_SITES:
        src = (REPO / rel).read_text()
        for i, line in enumerate(src.splitlines(), 1):
            if _VELOCITY_ALLOWANCE.search(line):
                bad.append(f"{rel}:{i}: {line.strip()[:80]}")
    assert not bad, (
        "velocity-derived projection allowance is back (A5.1/A7.3):\n"
        + "\n".join(bad)
        + "\n\nUse mpc_core.progress_gate.ProgressAllowanceGate.  The "
          "allowance must be banked PHYSICAL displacement projected on the "
          "tangent at the ACCEPTED arc; v*cos(e_psi) is measured against "
          "the RAW candidate, so a wrong lane grants itself permission."
    )


def test_no_local_allowed_ds_variable_in_ledgers():
    """``allowed_ds = ...`` means a ledger grew its own gate again."""
    bad = []
    for rel in LEDGER_SITES:
        src = (REPO / rel).read_text()
        for i, line in enumerate(src.splitlines(), 1):
            if re.match(r"\s*allowed_ds\s*=", line):
                bad.append(f"{rel}:{i}: {line.strip()[:80]}")
    assert not bad, (
        "local allowance recomputed instead of using the shared gate:\n"
        + "\n".join(bad)
    )


def test_every_ledger_uses_the_shared_gate():
    missing = [rel for rel in LEDGER_SITES
               if "ProgressAllowanceGate" not in (REPO / rel).read_text()]
    assert not missing, (
        "ledger(s) not routed through mpc_core.progress_gate: "
        + ", ".join(missing)
    )


def test_gate_constants_match_the_controller():
    """Ledger and controller must agree on what 'reachable' means.
    RECORDED ADAPTATION: the docx guard reads private ``_PROJECTION_MARGIN_M``
    constants in mpc.py; on this repo the controller's A5.1 constants live in
    MpcParams (registered numerically identical in R6), so the guard compares
    MpcParams defaults against the shared gate constants instead.
    """
    p = MpcParams()
    pairs = (("projection_margin_m", PROJECTION_MARGIN_M),
             ("allowance_cap_m", ALLOWANCE_CAP_M),
             ("odom_step_noise_m", ODOM_STEP_NOISE_M))
    for name, value in pairs:
        assert abs(getattr(p, name) - value) < 1e-12, (
            f"{name}: MpcParams has {getattr(p, name)}, progress_gate has {value}"
        )


# ------------------------------------------------------------- fixtures
def _straight_traj(length=10.0, n=501):
    s = np.linspace(0.0, length, n)
    return Trajectory(s=s, x=s.copy(), y=np.zeros(n), yaw=np.zeros(n),
                      kappa=np.zeros(n), v=np.full(n, 0.5))


def _folded_traj(length=2.0, offset=0.3, n=101):
    """Out along +x, back along -x at ``y = offset``: two antiparallel lanes.
    Arc length keeps increasing across the fold; the tangent flips by pi.
    """
    fx = np.linspace(0.0, length, n)
    rx = np.linspace(length, 0.0, n)
    x = np.concatenate([fx, rx])
    y = np.concatenate([np.zeros(n), np.full(n, offset)])
    yaw = np.concatenate([np.zeros(n), np.full(n, math.pi)])
    s = np.zeros(x.size)
    for i in range(1, x.size):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    return Trajectory(s=s, x=x, y=y, yaw=yaw, kappa=np.zeros(x.size),
                      v=np.full(x.size, 0.5))


# ------------------------------------------------------------- behaviour
def test_wrong_lane_cannot_grant_itself_budget():
    """A5.1(a): the tangent is taken at the ACCEPTED arc, never the raw one."""
    traj = _folded_traj()
    fold_s = traj.s[traj.x.size // 2]
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05, accepted_arc=1.0)
    gate.step(traj, 1.0, 0.0, 1.0)                 # anchor pose on lane A
    step_m = 0.04
    dec = gate.step(traj, 1.0 - step_m, 0.3, fold_s + 0.5)
    assert dec.physical_ds == 0.0, (
        "motion along the antiparallel lane banked budget against the "
        "outbound tangent"
    )
    # the same motion, banked at the RAW candidate's tangent (yaw = pi):
    raw_tangent_gain = (-step_m) * math.cos(math.pi)
    assert raw_tangent_gain > 0.0, "contrast fixture is wrong"
    assert not dec.accepted
    assert gate.accepted_arc == 1.0


def test_stationary_robot_banks_no_budget():
    """A5.1's anti-teleport pin, applied to the ledger."""
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    for _ in range(20):
        assert gate.step(traj, 0.0, 0.0, 0.0).accepted   # standing is no jump
    assert gate.budget == 0.0
    dec = gate.step(traj, 0.0, 0.0, 0.10)
    assert not dec.accepted, "stationary robot was granted a 0.10 m teleport"
    assert gate.accepted_arc == 0.0


def test_honest_tracking_is_never_rejected():
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)          # first call only anchors the pose
    x, rejected = 0.0, 0
    for _ in range(200):
        x += 0.04                           # 0.8 m/s, well above the margin
        rejected += int(not gate.step(traj, x, 0.0, x).accepted)
    assert rejected == 0, f"{rejected}/200 honest cycles rejected"
    assert abs(gate.accepted_arc - x) < 1e-9


def test_limit_grows_with_real_motion_after_a_rejection():
    """The limit tracks accumulated motion instead of a single instant."""
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    x, limits = 0.0, []
    for _ in range(5):
        x += 0.04
        dec = gate.step(traj, x, 0.0, 99.0)          # always over the limit
        assert not dec.accepted
        limits.append(dec.limit)
    assert all(b > a for a, b in zip(limits, limits[1:])), limits
    assert limits[-1] > limits[0] + 0.1


def test_lane_cut_jump_is_still_rejected():
    """Banking must not turn the gate into a rubber stamp."""
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    x = 0.0
    for _ in range(30):
        x += 0.04
        gate.step(traj, x, 0.0, x)
    honest = gate.accepted_arc
    dec = gate.step(traj, x + 0.04, 0.0, x + 3.0)     # anchor snaps 3 m ahead
    assert not dec.accepted
    assert gate.accepted_arc == honest


def test_localisation_jump_is_capped_not_banked():
    """A5.1(b): the excess of a teleport counts as ZERO, not a remainder."""
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    dec = gate.step(traj, 5.0, 0.0, 0.0)             # 5 m pose teleport
    assert dec.physical_ds == gate.step_cap
    assert gate.step_cap == 1.5 * 0.05 + ODOM_STEP_NOISE_M


def test_budget_is_capped_and_spent_on_accept():
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    x = 0.0
    for _ in range(200):                             # bank without accepting
        x += 0.04
        gate.step(traj, x, 0.0, 99.0)
    assert gate.budget <= ALLOWANCE_CAP_M + 1e-12
    assert gate.accepted_arc == 0.0
    dec = gate.step(traj, x, 0.0, 0.10)              # within margin + budget
    assert dec.accepted and gate.budget == 0.0


def test_baseline_is_forward_only():
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    x = 0.0
    for _ in range(30):
        x += 0.04
        gate.step(traj, x, 0.0, x)
    peak = gate.accepted_arc
    dec = gate.step(traj, x, 0.0, peak - 0.5)        # anchor drifts backward
    assert dec.accepted                               # backward is not a jump
    assert gate.accepted_arc == peak, "ledger baseline moved backwards"


def test_rebaseline_is_forward_only_and_spends_budget():
    traj = _straight_traj()
    gate = ProgressAllowanceGate(v_max=1.5, Ts=0.05)
    gate.step(traj, 0.0, 0.0, 0.0)
    gate.step(traj, 0.04, 0.0, 99.0)                 # rejected, budget banked
    assert gate.budget > 0.0
    gate.rebaseline(2.0)
    assert gate.accepted_arc == 2.0 and gate.budget == 0.0
    gate.rebaseline(1.0)
    assert gate.accepted_arc == 2.0
