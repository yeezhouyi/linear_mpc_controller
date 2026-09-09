#!/usr/bin/env python3
"""Reference-quality variants for the same waypoint plan (P0 comparison).

Generates two ALTERNATIVE dense references through the SAME waypoint chain
as the real u9 plan (90-degree boustrophedon corners), so that reference
QUALITY can be compared through the same MPC tracker:

  lattice_frenet   Apollo-lattice-INSPIRED: for every 90-degree corner,
                   sample candidate terminal states (blend half-length L on
                   the incoming and outgoing rows) and connect them with a
                   quintic Hermite curve (C2 tangents); pick the candidate
                   minimising comfort cost  int(kappa^2) ds + lambda * len.
  hybrid_astar     heading-aware grid search (8 headings, {straight,
                   +-45deg} one-cell primitives + in-place rotation --
                   differential drive) through the waypoint chain on a
                   0.05 m grid; dense-resampled to ds = 0.05.

Existing references for the same plan (already in results/a8_replay/geo/):
a8_cap.json (half-circle caps) and a8_nocap.json (straight 0.30 m jumps).

Output: results/ref_compare/geo/{lattice_frenet,hybrid_astar}.json in the
a8 geo traj format (x/y/yaw/kappa/s/v/segment_gear) directly loadable by
run_a8_matrix.load_traj.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from trajectory_tools.resample import resample_uniform  # noqa: E402

DS = 0.05
OMEGA_MAX = 2.0
V_CAP = 0.5
V_FLOOR = 0.15


def load_waypoints(path: str) -> list[tuple[float, float]]:
    rec = json.loads(Path(path).read_text())
    pts = [(float(p[0]), float(p[1])) for p in rec["poses"]]
    # drop consecutive duplicates
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-6:
            out.append(p)
    # simplify: keep only true corners (turn angle > 60 deg) + endpoints;
    # the raw u9 output is dense and already contains cap arcs, whose small
    # turns must NOT be re-blended by the lattice variant.
    if len(out) > 3:
        simp = [out[0]]
        for i in range(1, len(out) - 1):
            v1 = (out[i][0] - out[i - 1][0], out[i][1] - out[i - 1][1])
            v2 = (out[i + 1][0] - out[i][0], out[i + 1][1] - out[i][1])
            ang = abs(math.atan2(v1[0] * v2[1] - v1[1] * v2[0],
                                 v1[0] * v2[0] + v1[1] * v2[1]))
            if ang > math.radians(60.0):
                simp.append(out[i])
        simp.append(out[-1])
        out = simp
    return out


def finish(x, y, t_gen_ms: float, meta_extra: dict) -> dict:
    """Same completion rule as the a8 generator: yaw/kappa from chords,
    kinematic speed profile, all-forward gear."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = x.size
    s = np.zeros(n)
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
    yaw = np.zeros(n)
    for i in range(n):
        j0, j1 = max(0, i - 1), min(n - 1, i + 1)
        yaw[i] = math.atan2(y[j1] - y[j0], x[j1] - x[j0])
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        arc = 0.5 * ((s[i] - s[i - 1]) + (s[i + 1] - s[i]))
        if arc > 1e-9:
            kappa[i] = ((yaw[i + 1] - yaw[i - 1] + math.pi)
                        % (2 * math.pi) - math.pi) / (2 * arc)
    k_abs = np.abs(kappa)
    eps = 1e-6
    v = np.array([min(V_CAP, OMEGA_MAX / max(abs(k), eps)) for k in kappa])
    legal = k_abs <= OMEGA_MAX / max(V_FLOOR, 1e-9)
    v = np.where(legal, np.maximum(v, V_FLOOR), v)
    gear = np.ones(max(1, n - 1))
    k_rms = float(np.sqrt(np.mean(k_abs[1:-1] ** 2))) if n > 2 else 0.0
    lat_acc = float(np.max(k_abs * 0.5))  # peak lateral acc at v = 0.5
    return {
        "frame_id": "map",
        "meta": dict(kind=meta_extra.pop("kind"), gen_time_ms=t_gen_ms,
                     kappa_rms=round(k_rms, 4),
                     peak_lat_acc_at_vcap=round(lat_acc, 4), **meta_extra),
        "traj": {
            "x": [round(float(a), 6) for a in x],
            "y": [round(float(a), 6) for a in y],
            "yaw": [round(float(a), 10) for a in yaw],
            "kappa": [round(float(a), 10) for a in kappa],
            "s": [round(float(a), 10) for a in s],
            "v": [round(float(a), 10) for a in v],
            "segment_gear": [float(a) for a in gear],
        },
    }


# ------------------------------------------------------------------
# variant 1: lattice-style sampled corner blending
# ------------------------------------------------------------------
def _hermite_quintic(p0, t0, p1, t1, n: int):
    """Quintic Hermite: positions p0/p1, tangents |t| = L (scaled inside),
    zero 2nd-derivative at both ends.  Returns n points + per-point
    param s in [0, 1]."""
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    t0 = np.array(t0, dtype=float)
    t1 = np.array(t1, dtype=float)
    u = np.linspace(0.0, 1.0, n)
    h00 = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    h10 = u ** 3 - 6 * u ** 4 + 6 * u ** 5     # d/du factor absorbed below
    h01 = 1 - h00
    h11 = -4 * u ** 3 + 7 * u ** 4 - 3 * u ** 5
    # standard quintic Hermite basis (m10 = h10 for unit tangent scale)
    pts = (np.outer(h00, p0) + np.outer(h01, p1)
           + np.outer(h10, t0) + np.outer(h11, t1))
    return pts, u


def lattice_reference(wps, plan_meta: dict) -> dict:
    """Lattice-style sampled corner blending: for every corner sample the
    fillet radius R (terminal-state sampling in the corner parameter),
    score each candidate with the comfort cost  int(kappa^2) ds +
    0.02 * arc_length  subject to the straight-remainder constraint, and
    keep the argmin.  C1 corner treatment: curvature jumps at the
    tangent points are bounded by 1/R."""
    t0 = time.perf_counter()

    def corner_info(i):
        p_prev, p_corner, p_next = wps[i - 1], wps[i], wps[i + 1]
        d_in = np.array(p_corner) - np.array(p_prev)
        d_out = np.array(p_next) - np.array(p_corner)
        l_in = float(np.linalg.norm(d_in))
        l_out = float(np.linalg.norm(d_out))
        u_in = d_in / l_in
        u_out = d_out / l_out
        cosang = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
        turn = math.acos(cosang)          # turn angle at the corner
        return p_corner, u_in, u_out, turn, min(l_in, l_out)

    halves = []                            # kept for the meta record (R)
    corners = []
    for i in range(1, len(wps) - 1):
        p_corner, u_in, u_out, turn, lim = corner_info(i)
        if turn < 1e-6:
            halves.append(0.0)
            corners.append(None)
            continue
        best = None
        for radius in (0.05, 0.075, 0.1, 0.125, 0.15, 0.2, 0.3, 0.45, 0.6):
            tangent = radius / math.tan(turn / 2.0)
            if 2.0 * tangent > 1.0 * lim or tangent <= 0.0:
                continue
            cost = turn / radius + 0.02 * turn * radius   # comfort + length
            if best is None or cost < best[0]:
                best = (cost, radius, tangent, u_in, u_out, turn)
        if best is None:
            halves.append(0.0)
            corners.append(None)
            continue
        _cost, radius, tangent, u_in, u_out, turn = best
        halves.append(radius)
        a = np.array(p_corner) - tangent * u_in
        b = np.array(p_corner) + tangent * u_out
        # arc centre: a + R * normal_in (towards the inside of the turn)
        n_in = np.array([-u_in[1], u_in[0]])
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        if cross < 0:                      # right turn -> flip normal
            n_in = -n_in
        centre = a + radius * n_in
        a0 = math.atan2(a[1] - centre[1], a[0] - centre[0])
        a1 = math.atan2(b[1] - centre[1], b[0] - centre[0])
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        n = max(8, int(round(abs(da) * radius / DS)) + 1)
        arc = [(centre[0] + radius * math.cos(a0 + da * j / n),
                centre[1] + radius * math.sin(a0 + da * j / n))
               for j in range(n + 1)]
        corners.append(dict(a=a, b=b, arc=arc, tangent=tangent))

    def emit(p):
        if not xs or math.hypot(p[0] - xs[-1], p[1] - ys[-1]) > 1e-9:
            xs.append(float(p[0]))
            ys.append(float(p[1]))

    xs: list[float] = []
    ys: list[float] = []
    emit(np.array(wps[0]))
    prev_end = np.array(wps[0], dtype=float)
    for i in range(1, len(wps) - 1):
        p_corner, u_in, u_out, turn, _lim = corner_info(i)
        info = corners[i - 1]
        if info is None:
            a = np.array(p_corner)
            b = np.array(p_corner)
        else:
            a, b = info["a"], info["b"]
        seg = float(np.linalg.norm(a - prev_end))
        n = max(2, int(round(seg / DS)) + 1)
        for j in range(1, n + 1):
            emit(prev_end + (a - prev_end) * (j / (n - 1)))
        if info is not None:
            for p in info["arc"][1:]:
                emit(p)
        else:
            emit(np.array(p_corner))
        prev_end = b if info is not None else np.array(p_corner)
    seg = float(np.linalg.norm(np.array(wps[-1]) - prev_end))
    n = max(2, int(round(seg / DS)) + 1)
    for j in range(1, n + 1):
        emit(prev_end + (np.array(wps[-1]) - prev_end) * (j / (n - 1)))
    xr, yr = resample_uniform(np.array(xs), np.array(ys), ds=DS)
    t1 = time.perf_counter()
    return finish(xr, yr, (t1 - t0) * 1000.0,
                  {"kind": "lattice_frenet",
                   "fillet_radii": [round(c, 3) for c in halves if c > 0],
                   **plan_meta})


# ------------------------------------------------------------------
# variant 2: hybrid-A*-style heading-aware chain
# ------------------------------------------------------------------
def hybrid_reference(wps, plan_meta: dict, res: float = 0.05) -> dict:
    """8-heading grid search through the waypoint chain on a res-meter
    grid; primitives {straight, +-45deg} one-cell moves + in-place
    rotation (differential drive).  No obstacles: the u9 rect interior is
    free space; the variant isolates heading-feasible reference quality."""
    t0 = time.perf_counter()
    xs_all = [w[0] for w in wps]
    ys_all = [w[1] for w in wps]
    ox, oy = min(xs_all) - 0.5, min(ys_all) - 0.5
    w_cells = int(math.ceil((max(xs_all) - ox) / res)) + 1
    h_cells = int(math.ceil((max(ys_all) - oy) / res)) + 1
    moves = ((1, 0), (1, 1), (0, 1), (-1, 1),
             (-1, 0), (-1, -1), (0, -1), (1, -1))
    cells: list[tuple[int, int]] = []

    def to_cell(p):
        return (int(round((p[0] - ox) / res)), int(round((p[1] - oy) / res)))

    def to_world(c):
        return (ox + c[0] * res, oy + c[1] * res)

    cur = to_cell(wps[0])
    cells.append(cur)
    heading = 2 if (cur[0] + 1, cur[1]) else 0
    for wp in wps[1:]:
        goal = to_cell(wp)
        # A* over (x, y, heading); in-place turns allowed (diff-drive)
        start = (cur[0], cur[1], heading)
        import heapq
        dist = {start: 0.0}
        parent = {}
        open_list = [(0.0, start)]
        found = None
        while open_list:
            g, (cx, cy, ch) = heapq.heappop(open_list)
            if (cx, cy) == goal:
                found = (cx, cy, ch)
                break
            for dh in (-1, 0, 1):
                nh = (ch + dh) % 8
                ng = g + 0.4 + abs(dh) * 0.1        # in-place turn
                key = (cx, cy, nh)
                if key not in dist or ng < dist[key]:
                    dist[key] = ng
                    parent[key] = (cx, cy, ch)
                    hcost = math.hypot(goal[0] - cx, goal[1] - cy)
                    heapq.heappush(open_list, (ng + hcost, key))
                dy, dx = moves[nh]
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w_cells and 0 <= ny < h_cells):
                    continue
                ng2 = g + 1.0 + abs(dh) * 0.1
                key2 = (nx, ny, nh)
                if key2 not in dist or ng2 < dist[key2]:
                    dist[key2] = ng2
                    parent[key2] = (cx, cy, ch)
                    hcost = math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(open_list, (ng2 + hcost, key2))
        if found is None:
            raise RuntimeError("hybrid search failed between waypoints")
        chain = []
        key = found
        while key in parent:
            chain.append((key[0], key[1]))
            key = parent[key]
        chain.reverse()
        for c in chain[1:]:
            cells.append(c)
        cur = goal
        heading = found[2]
    pts = [to_world(c) for c in cells]
    # dedupe + dense resample
    ded = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - ded[-1][0], p[1] - ded[-1][1]) > 1e-9:
            ded.append(p)
    xr, yr = resample_uniform(np.array([p[0] for p in ded]),
                              np.array([p[1] for p in ded]), ds=DS)
    t1 = time.perf_counter()
    return finish(xr, yr, (t1 - t0) * 1000.0,
                  {"kind": "hybrid_astar", **plan_meta})


def synthetic_chain(rows: int = 6, row_len: float = 5.0,
                    lane_w: float = 0.3) -> list[tuple[float, float]]:
    """The SAME 6-row serpentine chain as the a8 geometries (row_len 5 m,
    lane 0.3 m) -- the fair comparison mission for cap/nocap."""
    wps = [(0.0, 0.0)]
    for k in range(rows):
        y = k * lane_w
        x_end = row_len if k % 2 == 0 else 0.0
        wps.append((x_end, y))            # sweep to the row end
        wps.append((x_end, y + lane_w))   # connector to the next lane
    return wps


def main() -> None:
    plan_path = sys.argv[1] if len(sys.argv) > 1 else "synthetic"
    outdir = Path(sys.argv[2] if len(sys.argv) > 2 else
                  "results/ref_compare/geo")
    outdir.mkdir(parents=True, exist_ok=True)
    if plan_path == "synthetic":
        wps = synthetic_chain()
    else:
        wps = load_waypoints(plan_path)
    meta = {"rows": len(wps), "source": plan_path}
    lat = lattice_reference(wps, dict(meta))
    (outdir / "lattice_frenet.json").write_text(json.dumps(lat))
    print("lattice_frenet: %d pts, L=%.2f m, gen=%.1f ms, blends=%s"
          % (len(lat["traj"]["x"]), lat["traj"]["s"][-1],
             lat["meta"]["gen_time_ms"], lat["meta"]["fillet_radii"]))
    hyb = hybrid_reference(wps, dict(meta))
    (outdir / "hybrid_astar.json").write_text(json.dumps(hyb))
    print("hybrid_astar: %d pts, L=%.2f m, gen=%.1f ms"
          % (len(hyb["traj"]["x"]), hyb["traj"]["s"][-1],
             hyb["meta"]["gen_time_ms"]))


if __name__ == "__main__":
    main()
