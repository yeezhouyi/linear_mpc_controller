#!/usr/bin/env python3
"""A5B.4 / A7.2 #5: diff the C++ projection chain against the SIGNED-IN golden.

The golden (mpc_core/tests/data/projection_golden.json) is committed to the
repository and is the single source of truth for both languages.  This script

  1. checks the C++ window/gate parameter defaults against the golden header
     (a silent drift of either side's defaults fails here, not in the field);
  2. replays each committed pose sequence through the C++ chain
     (the dump binary path is REQUIRED as argv[1]; CMake passes
     $<TARGET_FILE:projection_golden_dump>) and compares every record
     (stage / raw_arc / seg / e_y / accepted_arc) against the committed values.

A missing dump binary is an ERROR (exit 1) -- never a silent pass.  No
path argument at all is exit 2.
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
GOLDEN = ROOT / "mpc_core" / "tests" / "data" / "projection_golden.json"

TOL_ARC = 1e-6
TOL_EY = 1e-6

PARAMS = [
    "back_m", "fwd_m", "reacquire_m", "wide_m",
    "heading_gate_rad", "tie_eps_m",
    "projection_margin_m", "allowance_cap_m", "odom_step_noise_m",
]


def parse_params_header(line: str) -> dict:
    out = {}
    for tok in line.lstrip("#").split()[1:]:     # drop "params"
        k, _, v = tok.partition("=")
        out[k] = float(v)
    return out


def main(bin_path: pathlib.Path) -> int:
    if not bin_path.is_file():
        print(f"projection_golden: FAIL dump binary not found: {bin_path}")
        return 1
    golden = json.loads(GOLDEN.read_text())
    gparams = golden["params"]
    problems: list[str] = []

    with tempfile.TemporaryDirectory() as td:
        td = pathlib.Path(td)
        for name, records in golden["sequences"].items():
            poses = td / f"{name}_poses.txt"
            out = td / f"{name}_out.txt"
            poses.write_text(
                "\n".join(f"{p[0]} {p[1]} {p[2]}" for p in (r["pose"] for r in records))
            )
            subprocess.run([str(bin_path), name, str(poses), str(out)], check=True)
            lines = [ln for ln in out.read_text().splitlines() if ln.strip()]
            header = parse_params_header(lines[0])
            body = lines[1:]

            # 1) parameter defaults must match on both sides
            for k in PARAMS:
                if k in header and abs(header[k] - float(gparams[k])) > 1e-12:
                    problems.append(
                        f"param drift {k}: cpp={header[k]} golden={gparams[k]}")

            if len(body) != len(records):
                problems.append(
                    f"{name}: {len(body)} cpp records vs {len(records)} golden")
                continue
            for i, (ln, rec) in enumerate(zip(body, records)):
                stage, arc, seg, e_y, acc = ln.split()
                if int(stage) != int(rec["stage"]):
                    problems.append(
                        f"{name}[{i}] stage: cpp={stage} golden={rec['stage']}")
                if abs(float(arc) - float(rec["raw_arc"])) > TOL_ARC:
                    problems.append(
                        f"{name}[{i}] raw_arc: cpp={arc} golden={rec['raw_arc']}")
                if int(seg) != int(rec["seg"]):
                    problems.append(
                        f"{name}[{i}] seg: cpp={seg} golden={rec['seg']}")
                g_ey = -999.0 if rec["e_y"] is None else float(rec["e_y"])
                if abs(float(e_y) - g_ey) > TOL_EY:
                    problems.append(
                        f"{name}[{i}] e_y: cpp={e_y} golden={g_ey}")
                if abs(float(acc) - float(rec["accepted_arc"])) > TOL_ARC:
                    problems.append(
                        f"{name}[{i}] accepted_arc: cpp={acc} "
                        f"golden={rec['accepted_arc']}")

    total = sum(len(v) for v in golden["sequences"].values())
    if problems:
        print(f"projection golden: {len(problems)} mismatch(es) over {total} records")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"projection golden: C++ matches all {total} committed records "
          f"(params identical)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: check_projection_golden.py <projection_golden_dump>")
        sys.exit(2)
    sys.exit(main(pathlib.Path(sys.argv[1])))
