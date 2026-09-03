"""Frenet projection and error extraction (U2).

Given a robot pose and a dense reference ``Trajectory``, compute

* the reference anchor point (closest point projection with optional
  look-ahead), and
* the signed error state ``[e_y, e_psi, v, omega]``.

Sign conventions (frozen, see docs/mpc_model_derivation.md):
  ``e_y > 0`` means the robot is LEFT of the reference travel direction;
  ``e_psi = wrap(psi - psi_ref)``.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

from mpc_core.types import Trajectory, TrackPoint, KinematicState, wrap_angle


def _segments(traj: Trajectory) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Segment start/end points, tangent unit vectors, lengths."""
    dx = np.diff(traj.x)
    dy = np.diff(traj.y)
    lengths = np.hypot(dx, dy)
    # Degenerate segments (duplicate points) get tangent = previous tangent.
    tangents = np.stack(
        [
            np.where(lengths > 1e-12, dx / np.maximum(lengths, 1e-12), 0.0),
            np.where(lengths > 1e-12, dy / np.maximum(lengths, 1e-12), 0.0),
        ],
        axis=-1,
    )
    # fix zero tangents (straight repeats) by carrying forward
    for i in range(1, tangents.shape[0]):
        nz = np.hypot(tangents[i, 0], tangents[i, 1]) < 1e-12
        if nz:
            tangents[i] = tangents[i - 1]
    return dx, dy, lengths, tangents


def closest_point(
    traj: Trajectory, px: float, py: float
) -> Tuple[int, float, float, float]:
    """Brute-force nearest-segment projection (deterministic; O(n) with numpy).

    Returns ``(seg_idx, w, e_y, arc)`` where
      seg_idx : index of the segment start vertex (``seg_idx+1`` is the end),
      w       : interpolation weight inside the segment in [0, 1],
      e_y     : signed lateral error (positive LEFT of travel),
      arc     : arc-length coordinate of the projection.
    """
    dx, dy, lengths, tangents = _segments(traj)
    rel_x = px - traj.x[:-1]
    rel_y = py - traj.y[:-1]
    # project robot onto each segment axis
    tx = tangents[:, 0]
    ty = tangents[:, 1]
    along = rel_x * tx + rel_y * ty  # signed distance from segment start (clamped later)
    along_c = np.clip(along, 0.0, lengths)
    proj_x = traj.x[:-1] + along_c * tx
    proj_y = traj.y[:-1] + along_c * ty
    dist2 = (proj_x - px) ** 2 + (proj_y - py) ** 2
    seg = int(np.argmin(dist2))

    # normal vector: left normal of travel = rotate tangent by +90 deg
    nx_, ny_ = -ty[seg], tx[seg]
    lat = (px - proj_x[seg]) * nx_ + (py - proj_y[seg]) * ny_
    arc = traj.s[seg] + along_c[seg]
    w = float(along_c[seg] / lengths[seg]) if lengths[seg] > 1e-12 else 0.0
    return seg, min(max(w, 0.0), 1.0), float(lat), float(arc)


def frenet_state(
    traj: Trajectory,
    state: KinematicState,
    lookahead_m: float = 0.0,
) -> Tuple[TrackPoint, np.ndarray]:
    """Error state relative to the reference.

    * projects the pose onto the path,
    * optionally shifts the anchor forward by ``lookahead_m`` along the path
      (compensates discrete-time delay; documented in the derivation doc),
    * returns ``(anchor, err)`` with ``err = [e_y, e_psi, v, omega]`` where
      the v/omega entries are the *absolute* robot velocities (the reference
      velocity/curvature enter the MPC through the reference state, not here).

    Deterministic behaviour on the ends: clamping, never wraps around.
    """
    _, _, e_y, arc = closest_point(traj, state.x, state.y)
    if lookahead_m > 0.0:
        arc = min(arc + lookahead_m, traj.s[-1])
    anchor = traj.sample_by_s(arc)
    e_psi = wrap_angle(state.yaw - anchor.yaw)
    err = np.array([e_y, e_psi, state.v, state.omega], dtype=float)
    return anchor, err


def predicted_anchor_by_s(
    traj: Trajectory,
    base_arc: float,
    s_ahead: float,
) -> TrackPoint:
    """Reference anchor at ``base_arc + s_ahead`` (used for the horizon window)."""
    return traj.sample_by_s(min(base_arc + s_ahead, traj.s[-1]))
