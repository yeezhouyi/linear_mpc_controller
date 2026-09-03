"""Linear MPC trajectory tracking controller -- ROS-free Python reference core.

This package mirrors the C++/Eigen core in ``include/linear_mpc_controller`` and
``src/`` (which runs in ROS2/WSL2).  It is the *executable reference* used for:

* offline unit tests and math validation (see ``mpc_core/tests``),
* the fast RL environment in ``mpc_rl_env``,
* trajectory/benchmark tooling in ``trajectory_tools`` / ``benchmark_tools``.

The authoritative math derivation and frozen conventions live in
``docs/mpc_model_derivation.md``.
"""
from mpc_core.types import (
    HealthState,
    KinematicState,
    MpcDiagnostics,
    MpcOutput,
    MpcParams,
    TrackPoint,
    TrackPointKind,
    Trajectory,
)

__all__ = [
    "HealthState",
    "KinematicState",
    "MpcDiagnostics",
    "MpcOutput",
    "MpcParams",
    "TrackPoint",
    "TrackPointKind",
    "Trajectory",
]

__version__ = "0.3.0"
