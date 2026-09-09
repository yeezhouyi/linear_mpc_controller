#!/usr/bin/env python3
"""Reference-quality comparison through the SAME MPC tracker (P0).

For each dense reference through the same u9 waypoint chain:
  a8_cap          half-circle caps (repo/u9 style)
  a8_nocap        straight 0.30 m jumps (pre-u9 style)
  lattice_frenet  Apollo-lattice-inspired sampled quintic blends
  hybrid_astar    heading-aware grid chain (8 headings, diff-drive)

...run the 4 matrix cells (v_min x controller_projection_mode) of the
accepted-arc auditor and report tracking success + accuracy + comfort +
planner generation cost in one table.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from benchmark_tools.scripts.run_a8_matrix import load_traj, run_cell  # noqa: E402

GEOS = ("a8_cap", "a8_nocap", "lattice_frenet", "hybrid_astar")
GEO_DIRS = {
    "a8_cap": "results/a8_replay/geo",
    "a8_nocap": "results/a8_replay/geo",
    "lattice_frenet": "results/ref_compare/geo",
    "hybrid_astar": "results/ref_compare/geo",
}
V_MINS = (0.0, -0.5)
MODES = (("stateful_windowed", "windowed"), ("global_legacy", "global"))


def comfort(traj) -> dict:
    k = np.abs(traj.kappa)
    v = np.abs(traj.v)
    lat = v ** 2 * k                       # lateral acceleration per point
    ds = np.diff(traj.s)
    dv = np.diff(np.abs(traj.v))
    dt = np.maximum(ds / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-6), 1e-6)
    acc = dv / dt                          # longitudinal acceleration
    return dict(
        kappa_rms=round(float(np.sqrt(np.mean(k ** 2))), 4),
        peak_kappa=round(float(k.max()), 3),
        lat_acc_rms=round(float(np.sqrt(np.mean(lat ** 2))), 4),
        lon_acc_rms=round(float(np.sqrt(np.mean(acc ** 2))), 4),
    )


def main() -> None:
    outdir = Path("results/ref_compare")
    outdir.mkdir(parents=True, exist_ok=True)
    rows = []
    refs = {}
    for g in GEOS:
        traj = load_traj(Path(GEO_DIRS[g]) / (g + ".json"))
        meta = json.loads((Path(GEO_DIRS[g]) / (g + ".json")).read_text())["meta"]
        refs[g] = dict(traj=traj, meta=meta)
        print("%-16s L=%6.2f m gen=%7.1f ms kappa_rms=%.4f peak_k=%.2f"
              % (g, traj.s[-1], meta.get("gen_time_ms", float("nan")),
                 comfort(traj)["kappa_rms"], comfort(traj)["peak_kappa"]),
              flush=True)
    for g in GEOS:
        traj = refs[g]["traj"]
        for v_min in V_MINS:
            for mode_doc, mode_code in MODES:
                name = "%s|v_min=%s|%s" % (g, v_min, mode_doc)
                print("=== cell", name, flush=True)
                c = run_cell(traj, name, v_min, mode_doc, mode_code,
                             "accepted", 8000)
                c["reference"] = g
                c["gen_time_ms"] = refs[g]["meta"].get("gen_time_ms")
                c["comfort"] = comfort(traj)
                (outdir / ("cell_%s_%s_%s.json" % (g, v_min, mode_code))
                 ).write_text(json.dumps(c, indent=1))
                rows.append(c)
                print(json.dumps({k: c[k] for k in (
                    "done_reason", "progress_ratio", "e_y_rms",
                    "qp_failures")}), flush=True)
    # ---- aggregate: medians over the 4 cells per reference ----
    import statistics
    agg = {}
    for g in GEOS:
        sel = [r for r in rows if r["reference"] == g]
        agg[g] = dict(
            completed=sum(1 for r in sel if r["done_reason"] == "COMPLETED"),
            progress=round(statistics.median(r["progress_ratio"] for r in sel), 4),
            e_y_rms=round(statistics.median(r["e_y_rms"] for r in sel), 4),
            qp_failures=statistics.median(r["qp_failures"] for r in sel),
            gen_time_ms=round(refs[g]["meta"].get("gen_time_ms", -1), 1),
            ref_length_m=round(float(refs[g]["traj"].s[-1]), 2),
            **comfort(refs[g]["traj"]),
        )
    (outdir / "ref_compare.json").write_text(json.dumps(agg, indent=1))

    md = ["# Reference-quality comparison through the SAME MPC tracker", "",
          "Same u9 waypoint chain; 4 matrix cells per reference (v_min x",
          "projection mode); medians over cells.", "",
          "| reference | L (m) | completed/4 | progress | e_y_rms | qp_fail"
          " | kappa_rms | peak_kappa | lat_acc_rms | gen_ms |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for g in GEOS:
        a = agg[g]
        md.append("| %s | %.2f | %d/4 | %.4f | %.4f | %.0f | %.4f | %.2f"
                  " | %.4f | %.1f |"
                  % (g, a["ref_length_m"], a["completed"], a["progress"],
                     a["e_y_rms"], a["qp_failures"], a["kappa_rms"],
                     a["peak_kappa"], a["lat_acc_rms"], a["gen_time_ms"]))
    (outdir / "ref_compare.md").write_text("\n".join(md) + "\n")
    print("written:", outdir / "ref_compare.md")
    print(json.dumps(agg, indent=1))


if __name__ == "__main__":
    main()
