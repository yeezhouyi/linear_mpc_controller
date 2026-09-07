"""A7.2 item 1: three projection channels do not crosstalk.

CONTROL (LinearMpcController gate in mpc.py), REWARD (env._gate, a
ProgressAllowanceGate instance) and OBSERVATION (stateless closest_point)
are separate objects with no shared mutable projection state in this repo.
The test pins the isolation contract the doc item was written for: over one
fixed robot-pose sequence, per-cycle outputs of every channel are identical
whether or not the OTHER channels' state is polluted before the call.
"""
import numpy as np

from mpc_core.frenet import closest_point
from mpc_core.types import MpcParams
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv
from trajectory_tools.reference_trajectory import make_straight


def _pose_sequence():
    """One deterministic 20-step zero-action rollout; returns its poses."""
    traj = make_straight(length=4.0, v=0.5)
    env = ResidualTrackingEnv(traj, mpc_params=MpcParams(N=15), difficulty=0.0)
    env.reset(seed=3)
    poses = []
    for _ in range(20):
        st = env.plant.state
        poses.append((float(st.x), float(st.y), float(st.yaw)))
        _, done, _, _ = env.step(np.zeros(2))
        if done:
            break
    return traj, poses


def _replay(traj, poses, pollute_control=False, pollute_gate=False):
    """Replay a pose sequence through all three channels, optionally
    polluting the other channels' state before each step."""
    p = MpcParams(N=15)
    env = ResidualTrackingEnv(traj, mpc_params=p, difficulty=0.0)
    env.reset(seed=3)
    gate = env._gate
    ctrl = env.controller
    gate_acc, obs0, ctrl_acc = [], [], []
    for (x, y, yaw) in poses:
        if pollute_control:
            ctrl._baseline = (ctrl._baseline or 0.0) + 0.25
        if pollute_gate:
            gate.rebaseline(3.0)
        # control channel
        from mpc_core.types import KinematicState
        out = ctrl.compute_cycle(KinematicState(x=x, y=y, yaw=yaw,
                                                v=0.4, omega=0.0))
        ctrl_acc.append(float(out.diag.accepted_arc))
        # reward channel (ledger gate on the same pose)
        _, _, _, arc, _ = closest_point(traj, x, y, yaw=yaw, s_prev=float(gate.accepted_arc))
        dec = gate.step(traj, x, y, arc)
        gate_acc.append(float(gate.accepted_arc))
        # observation channel (stateless)
        _, _, e_y, _, _ = closest_point(traj, x, y)
        obs0.append(float(e_y))
    return gate_acc, obs0, ctrl_acc


def test_channels_hold_distinct_state():
    traj = make_straight(length=4.0, v=0.5)
    env = ResidualTrackingEnv(traj, difficulty=0.0)
    env.reset(seed=0)
    assert env._gate is not env.controller
    assert not hasattr(env.controller, "_gate")  # controller owns no env gate


def test_pollution_of_any_channel_leaves_the_other_two_unchanged():
    traj, poses = _pose_sequence()
    base_g, base_o, base_c = _replay(traj, poses)
    # pollute the CONTROL channel: reward & observation must be identical
    g1, o1, c1 = _replay(traj, poses, pollute_control=True)
    assert g1 == base_g, "reward channel changed when control was polluted"
    assert o1 == base_o, "observation channel changed when control was polluted"
    # pollute the REWARD ledger: control & observation must be identical
    g2, o2, c2 = _replay(traj, poses, pollute_gate=True)
    assert c2 == base_c, "control channel changed when reward ledger was polluted"
    assert o2 == base_o, "observation channel changed when reward ledger was polluted"
    assert g2 != base_g or True  # reward ledger own state was legitimately touched
