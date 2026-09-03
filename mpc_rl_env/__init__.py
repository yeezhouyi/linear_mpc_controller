"""mpc_rl_env: fast residual-RL environment for the MPC trajectory tracker.

The environment wraps :class:`LinearMpcController` (reference core) and a
:class:`DifferentialDrivePlant`; the policy action is a *residual* on the MPC
output, projected by :mod:`mpc_rl_env.algorithms.safety_projection`.
Gymnasium is optional (used by train/eval entry points); the core Env class
is dependency-free (numpy only) so contract tests run anywhere.
"""
from mpc_rl_env.envs.fast_tracking_env import ResidualTrackingEnv  # noqa: F401

__all__ = ["ResidualTrackingEnv"]
