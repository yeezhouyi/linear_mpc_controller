"""Fast residual-RL environment (U8/C6): MPC + plant + residual action.

Design notes (R14-R19):
* state  : observation vector (see observation_builder) -- runtime only,
* action : normalised residual ``delta_u in [-1,1]^2`` (NOT full cmd_vel),
* step   : ``raw = u_mpc + alpha * scale(delta_u)``, then SafetyProjection,
* plant  : unicycle with optional first-order lag / delay / noise (profile),
* reward : error + progress + smoothness + constraint/collision/stall terms.

Gymnasium is optional: the env itself is dependency-free.  Training entry
points (train_ppo_residual.py) build a gymnasium wrapper when available.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from mpc_core.frenet import closest_point
from mpc_core.model import DifferentialDrivePlant
from mpc_core.mpc import LinearMpcController
from mpc_core.types import HealthState, KinematicState, MpcParams, Trajectory, wrap_angle
from mpc_rl_env.algorithms.safety_projection import SafetyProjection
from mpc_rl_env.envs.observation_builder import build_observation
from mpc_rl_env.envs.randomization import RandomizationProfile, sample_profile
from mpc_rl_env.envs.reward import RewardWeights, compute_reward
from mpc_rl_env.envs.termination import TerminationConfig, check_termination

ACTION_DV_MAX = 0.20   # m/s  per step at full action
ACTION_DW_MAX = 0.50   # rad/s per step at full action


class ResidualTrackingEnv:
    """Fast env: identical discrete model family as the MPC core."""

    def __init__(
        self,
        traj: Trajectory,
        mpc_params: Optional[MpcParams] = None,
        reward_w: Optional[RewardWeights] = None,
        term_cfg: Optional[TerminationConfig] = None,
        alpha_residual: float = 1.0,
        difficulty: float = 0.3,
    ) -> None:
        self.traj = traj
        self.mpc_params = mpc_params or MpcParams()
        self.controller = LinearMpcController(self.mpc_params, traj)
        self.reward_w = reward_w or RewardWeights()
        self.term_cfg = term_cfg or TerminationConfig()
        self.alpha = float(alpha_residual)
        self.difficulty = float(difficulty)
        self.projection = SafetyProjection(
            v_min=self.mpc_params.v_min,
            v_max=self.mpc_params.v_max,
            omega_max=self.mpc_params.omega_max,
            dv_max_per_step=self.mpc_params.a_max * self.mpc_params.Ts,
            dw_max_per_step=self.mpc_params.alpha_max * self.mpc_params.Ts,
        )
        self.rng = np.random.default_rng(0)
        self.profile: RandomizationProfile = RandomizationProfile()
        self.plant = DifferentialDrivePlant()
        self.step_count = 0
        self._prev_err = np.zeros(4)
        self._prev_cmd = np.zeros(2)
        self._arc_hist: list = []
        self._delay_buf: list = []
        self._obs = np.zeros(12)

    # -- MDP API (gymnasium-free) -------------------------------------------
    def reset(self, seed: Optional[int] = None, profile: Optional[RandomizationProfile] = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.profile = profile or sample_profile(self.rng, self.traj_kind(), self.difficulty)
        r = self.profile.rng if profile is None else self.rng
        # initial condition from the profile + track start pose
        start = self.traj.at_index(0)
        x0 = start.x + float(r.uniform(-0.1, 0.1))
        y0 = start.y + self.profile.initial_lateral_m
        yaw0 = wrap_angle(start.yaw + self.profile.initial_heading_rad)
        self.plant = DifferentialDrivePlant(
            x0=x0, y0=y0, yaw0=yaw0, v0=0.0, omega0=0.0, lag_s=self.profile.velocity_lag_s
        )
        self.controller = LinearMpcController(self.mpc_params, self.traj)
        self.controller.set_reference(self.traj)
        self.step_count = 0
        self._prev_err = np.zeros(4)
        self._prev_cmd = np.zeros(2)
        self._arc_hist = []
        self._delay_buf = []
        self._obs = self._observe()
        return self._obs.copy(), {}

    def step(self, action: np.ndarray):
        """action: normalised residual in [-1, 1]^2."""
        action = np.clip(np.asarray(action, dtype=float).ravel(), -1.0, 1.0)
        self.step_count += 1

        # 1) MPC baseline command (pure MPC on the observed state)
        st = self.plant.state
        out = self.controller.compute_cycle(st)
        u_mpc = np.array([out.v_cmd, out.omega_cmd])

        # 2) residual fusion + safety projection
        delta = np.array([action[0] * ACTION_DV_MAX, action[1] * ACTION_DW_MAX])
        cmd = self.projection.project(u_mpc, delta, self.alpha, self._prev_cmd)

        # 3) command delay (integer steps) then plant step
        self._delay_buf.append(cmd.copy())
        applied = cmd
        if len(self._delay_buf) > self.profile.control_delay_steps + 1:
            applied = self._delay_buf.pop(0)
        self.plant.step(applied[0], applied[1], self.mpc_params.Ts)

        # 4) metrics & termination
        st = self.plant.state
        _, _, e_y, arc = closest_point(self.traj, st.x, st.y)
        anchor = self.traj.sample_by_s(arc)
        e_psi = wrap_angle(st.yaw - anchor.yaw)
        err = np.array([e_y, e_psi, st.v, st.omega])
        du_norm2 = float(np.sum((cmd - self._prev_cmd) ** 2))
        ds = max(0.0, arc - (self._arc_hist[-1] if self._arc_hist else 0.0))
        self._arc_hist.append(arc)
        stalled = st.v < self.reward_w.stall_threshold_v and self.step_count > 20

        terminated, reason = check_termination(
            self.term_cfg, self.step_count, arc, self._arc_hist, e_y,
            self.traj.total_length, out.diag.health,
        )
        # measure noise is applied on the *observed* state only (next obs)
        self._prev_err = err
        self._prev_cmd = cmd

        reward = compute_reward(
            self.reward_w, e_y, e_psi, st.v - anchor.v, du_norm2, ds,
            collision=False, constraint_violation=out.diag.constraint_violation,
            stalled=stalled, terminated=terminated,
            completed=(reason == "completed"),
        )
        self._obs = self._observe()
        info = {"reason": reason, "e_y": float(e_y), "health": int(out.diag.health)}
        return self._obs.copy(), float(reward), bool(terminated), info

    def _observe(self) -> np.ndarray:
        st = self.plant.state
        # add pose measurement noise from the profile (odom-like)
        if self.profile.measure_noise_m > 0 and self.step_count > 0:
            nx = st.x + self.profile.rng.normal(0.0, self.profile.measure_noise_m)
            ny = st.y + self.profile.rng.normal(0.0, self.profile.measure_noise_m)
            _, _, e_y, arc = closest_point(self.traj, nx, ny)
            anchor = self.traj.sample_by_s(arc)
            e_psi = wrap_angle(st.yaw - anchor.yaw)
            err = np.array([e_y, e_psi, st.v, st.omega])
        else:
            _, _, e_y, arc = closest_point(self.traj, st.x, st.y)
            anchor = self.traj.sample_by_s(arc)
            e_psi = wrap_angle(st.yaw - anchor.yaw)
            err = np.array([e_y, e_psi, st.v, st.omega])
        edot = (err - self._prev_err) / max(self.mpc_params.Ts, 1e-9)
        obs = build_observation(err, anchor.v, anchor.kappa, self._prev_cmd,
                                mpc_health_ok=True, obstacle_cost=0.0)
        obs[6] = float(np.clip(edot[0], -5.0, 5.0))
        obs[7] = float(np.clip(edot[1], -5.0, 5.0))
        return obs

    def traj_kind(self) -> str:
        return getattr(self.traj, "kind", "unknown")
