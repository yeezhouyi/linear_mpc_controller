"""Projection-sequence golden test (doc A7.2 item 5, Python side).

Regenerates every committed sequence with the CURRENT code and compares it
bit-for-bit against the signed-in file (mpc_core/tests/data/
projection_golden.json).  Any drift of projection outputs -- windowing,
heading gate, tie/stage semantics, gate acceptance -- trips this test.  The
params header is also checked against the live function defaults and the
MpcParams A5.1 constants, so a changed default on either side goes red.

The C++ side (A5B) does not exist in this repo yet; when it lands it must
compare against the SAME file, not against the Python side (recorded).
"""
import json
from pathlib import Path

from mpc_core.tools.generate_projection_golden import (
    SEQUENCES, build_golden, defaults_header, generate_sequence,
)

GOLDEN_PATH = (Path(__file__).resolve().parents[1] / "tests" / "data"
               / "projection_golden.json")


def test_golden_file_exists_and_has_all_sequences():
    assert GOLDEN_PATH.exists()
    golden = json.loads(GOLDEN_PATH.read_text())
    assert set(golden["sequences"]) == set(SEQUENCES)


def test_regeneration_matches_committed_golden():
    golden = json.loads(GOLDEN_PATH.read_text())
    fresh = build_golden()
    assert fresh == golden, (
        "projection outputs drifted from the committed golden file:\n"
        "  run:  mpc_core/tools/generate_projection_golden.py\n"
        "  diff the regenerated JSON against mpc_core/tests/data/"
        "projection_golden.json"
    )


def test_golden_params_header_matches_live_defaults():
    golden = json.loads(GOLDEN_PATH.read_text())
    assert golden["params"] == defaults_header(), (
        "projection / gate parameter defaults drifted from the golden header"
    )


def test_each_sequence_records_all_steps():
    golden = json.loads(GOLDEN_PATH.read_text())
    for name in SEQUENCES:
        _, records = generate_sequence(name)
        assert len(records) == len(golden["sequences"][name])
