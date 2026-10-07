"""Explicit project-owned T07 v1 training adapter.

The legacy fast environment remains the default caller path.  This adapter
must be selected by name and records a JSON-safe manifest for every run.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from typing import Callable, Mapping, Sequence

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from mpc_core.mpc import LinearMpcController
from mpc_core.types import KinematicState, MpcParams
from trajectory_tools.reference_trajectory import generate_benchmark_tracks, integrate_segments

try:
    from tools.t07_contract_env import (ACTION_DIM, CONTRACT, OBS_DIM, T07Config,
                                        T07FastEnv)
except ModuleNotFoundError:  # direct execution from the tools directory
    from t07_contract_env import (ACTION_DIM, CONTRACT, OBS_DIM, T07Config,
                                  T07FastEnv)

ADAPTER_VERSION = "project_t07_adapter_v1"

# ``--track`` names are the benchmark names used by the MPC and evaluation
# tools. The two arc aliases are retained for the original T07 contract tests
# and are mapped to explicit synthetic trajectories.
_TRACK_ALIASES = {
    "straight": "straight",
    "circle": "circle",
    "s_curve": "s_curve",
    "u_turn": "u_turn",
    "left_arc": "left_arc",
    "right_arc": "right_arc",
}


def resolve_track_id(path_id: str) -> str:
    """Validate and canonicalize a T07 path identifier."""
    key = str(path_id).strip().lower()
    try:
        return _TRACK_ALIASES[key]
    except KeyError as exc:
        supported = ", ".join(sorted(_TRACK_ALIASES))
        raise ValueError(
            f"unsupported T07 path_id {path_id!r}; expected one of {supported}"
        ) from exc


def _build_callback_tracks() -> dict[str, object]:
    """Return benchmark trajectories plus explicit T07 arc aliases."""
    tracks = dict(generate_benchmark_tracks())
    # T07's legacy aliases use v=0.15 and radius=2 m (kappa=+/-0.5).
    tracks["left_arc"] = integrate_segments(
        [{"kind": "arc", "length": 4.0 * np.pi, "kappa": 0.5, "v": 0.15}]
    )
    tracks["right_arc"] = integrate_segments(
        [{"kind": "arc", "length": 4.0 * np.pi, "kappa": -0.5, "v": 0.15}]
    )
    return tracks


class BenchmarkMpcCallback:
    """Callable T07 baseline backed by the project LinearMpcController."""

    def __init__(self, config: T07Config | None = None) -> None:
        self.config = config or T07Config()
        self.config.validate()
        params = MpcParams(
            Ts=self.config.dt_s,
            N=25,
            v_max=self.config.v_limit,
            omega_max=self.config.omega_limit,
        )
        self._tracks = _build_callback_tracks()
        self._controllers = {
            name: LinearMpcController(params, traj)
            for name, traj in self._tracks.items()
        }
        self.calls = 0
        self.last_path_id: str | None = None

    @property
    def track_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._tracks))

    def __call__(self, measurement: Mapping[str, float]) -> tuple[float, float]:
        path_id = resolve_track_id(measurement.get("path_id", self.config.path_id))
        controller = self._controllers[path_id]
        state = KinematicState(
            x=float(measurement.get("x", 0.0)),
            y=float(measurement.get("y", 0.0)),
            yaw=float(measurement.get("yaw", 0.0)),
            v=float(measurement.get("v", 0.0)),
            omega=float(measurement.get("omega", 0.0)),
        )
        out = controller.compute_cycle(state)
        self.calls += 1
        self.last_path_id = path_id
        command = (float(out.v_cmd), float(out.omega_cmd))
        if not np.all(np.isfinite(command)):
            raise RuntimeError(
                f"LinearMpcController produced non-finite command for {path_id}"
            )
        return command


class BoxSpec:
    """Dependency-free Box metadata used by offline checks."""

    def __init__(self, low: float, high: float, shape: tuple[int, ...]) -> None:
        self.low = np.full(shape, low, dtype=np.float32)
        self.high = np.full(shape, high, dtype=np.float32)
        self.shape = shape
        self.dtype = np.dtype(np.float32)

    def contains(self, value: Sequence[float] | np.ndarray) -> bool:
        arr = np.asarray(value)
        return bool(arr.shape == self.shape and np.all(np.isfinite(arr))
                    and np.all(arr >= self.low) and np.all(arr <= self.high))


class T07TrainingAdapter:
    """Expose frozen T07 spaces and preserve inner step semantics."""

    def __init__(self, *, contract: str = CONTRACT, config: T07Config | None = None,
                 mpc: Callable[[Mapping[str, float]], Sequence[float]] | None = None,
                 run_name: str = "offline_t07") -> None:
        if contract != CONTRACT:
            raise ValueError(f"unsupported adapter contract {contract!r}; "
                             "legacy training remains outside this adapter")
        self.contract = contract; self.run_name = str(run_name)
        self.config = config or T07Config()
        # The default T07 baseline is the real project MPC. Tests and offline
        # mechanism probes may still inject a deterministic callback explicitly.
        self.mpc = mpc if mpc is not None else BenchmarkMpcCallback(self.config)
        self.inner = T07FastEnv(self.config, mpc=self.mpc)
        self.observation_dim = OBS_DIM; self.action_dim = ACTION_DIM
        self.observation_space = BoxSpec(-np.inf, np.inf, (OBS_DIM,))
        self.action_space = BoxSpec(-1.0, 1.0, (ACTION_DIM,))
        self._last_seed: int | None = None

    def reset(self, seed: int | None = None, *, path_id: str | None = None):
        self._last_seed = seed
        if path_id is not None:
            resolve_track_id(path_id)
        obs, info = self.inner.reset(seed=seed, path_id=path_id)
        info = dict(info); info.update(self._adapter_info())
        return np.asarray(obs, dtype=np.float32), info

    def step(self, action: Sequence[float] | np.ndarray):
        obs, reward, done, info = self.inner.step(action)
        info = dict(info); info.update(self._adapter_info())
        return np.asarray(obs, dtype=np.float32), float(reward), bool(done), info

    def _adapter_info(self) -> dict[str, object]:
        return {"contract": self.contract, "adapter_version": ADAPTER_VERSION,
                "observation_dim": self.observation_dim, "action_dim": self.action_dim,
                "legacy_mode": False, "path_id": self.inner.path_id,
                "baseline": "LinearMpcController" if isinstance(self.mpc, BenchmarkMpcCallback)
                else "injected_callback"}

    def manifest(self, *, seed: int | None = None) -> dict[str, object]:
        effective = dict(self.inner.effective_parameters)
        config_dict = asdict(self.config)
        config_json = json.dumps(config_dict, sort_keys=True, separators=(",", ":"))
        return {
            "manifest_version": 1, "adapter_version": ADAPTER_VERSION,
            "contract": self.contract, "run_name": self.run_name,
            "seed": self._last_seed if seed is None else seed,
            "observation": {"shape": [self.observation_dim], "dtype": "float32",
                            "features": effective["observation_features"],
                            "history_length": effective["history_length"],
                            "normalization_std": effective["observation_normalization_std"]},
            "action": {"shape": [self.action_dim], "dtype": "float32",
                       "normalized_range": [-1.0, 1.0],
                       "delta_omega_scale_radps": effective["action_delta_omega_scale"]},
            "effective_parameters": effective, "config": config_dict,
            "config_sha256": hashlib.sha256(config_json.encode()).hexdigest(),
            "baseline": ("LinearMpcController" if isinstance(self.mpc, BenchmarkMpcCallback)
                          else "injected_callback"),
            "benchmark_tracks": (list(self.mpc.track_ids)
                                  if isinstance(self.mpc, BenchmarkMpcCallback) else []),
            "canonical_checkout": "read_only",
            "legacy_compatibility": "legacy 12D/2D entry point is unchanged and opt-in separately",
        }

    def write_manifest(self, path: str | Path, *, seed: int | None = None) -> Path:
        output = Path(path); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.manifest(seed=seed), indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
        return output


def make_training_adapter(*, contract: str, **kwargs) -> T07TrainingAdapter:
    """Require explicit contract selection to prevent legacy replacement."""
    return T07TrainingAdapter(contract=contract, **kwargs)


__all__ = ["ADAPTER_VERSION", "CONTRACT", "BenchmarkMpcCallback", "BoxSpec",
           "T07TrainingAdapter", "make_training_adapter", "resolve_track_id"]
