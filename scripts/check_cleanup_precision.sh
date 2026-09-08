#!/usr/bin/env bash
# B6B cleanup precision check -- PAIRED assertions (red & green both
# required; a test with only the NEG side is vacuous in the failure
# direction, because an all-no-op cleanup also leaves the unrelated proc
# alive):
#
#   NEG : unrelated proc whose argv contains "controller_server" but NOT our
#         CFG            -> must SURVIVE both cleanups (not over-broad)
#   POS : genuine target whose argv CONTAINS our CFG and sits off the
#         runner's ancestor chain -> MUST be killed (cleanup actually fires)
#
# Usage: check_cleanup_precision.sh [runner]
#   Default runner = run_b6b_sandbox.sh NEXT TO THIS SCRIPT (derived from
#   $0, never a hardcoded absolute path -- a no-arg invocation must test
#   the runner this file ships with, not some other checkout).
# CFG is parsed from THE RUNNER UNDER TEST (its own WS/REPO/CFG lines),
# never a hardcoded copy: if the runner's config path moves and this file
# still carried the old string, the POS target would never match -> false
# red -> and the natural "fix" (loosen the match) dismantles the exact
# precision this test protects.
# Exit 0 = both assertions hold AND the gate itself passed.
set -u
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
RUNNER=${1:-"$SCRIPT_DIR/run_b6b_sandbox.sh"}

[ -f "$RUNNER" ] || { echo "FAIL: runner not found: $RUNNER"; exit 1; }
eval "$(grep -E '^(WS|REPO|CFG)=' "$RUNNER" | head -3)"
CFG_ACTUAL=${CFG:-}
[ -n "$CFG_ACTUAL" ] || { echo "FAIL: cannot derive CFG from runner $RUNNER"; exit 1; }
echo "runner under test: $RUNNER"
echo "cfg derived from runner: $CFG_ACTUAL"

# NEG: argv[0] renamed, args carry no CFG path
exec -a controller_server_dummy sleep 90 &
NEG=$!
# POS: a process whose argv STABLY contains our CFG and sits off the
# runner's ancestor chain -> MUST be killed (cleanup actually fires).
# Pitfall: `bash -c 'sleep 90' sh "$CFG"` is a BAD target -- bash exec-
# optimizes a single-command -c script and replaces itself with `sleep 90`
# within ~0.25 s, wiping CFG from argv (the fixed cleanup then correctly
# spares it, giving a FALSE red on the POS side).  python3 keeps its argv
# (the CFG path lands in sys.argv) for the whole lifetime.
python3 -c 'import time; time.sleep(90)' "$CFG_ACTUAL" &
POS=$!
echo "neg pid=$NEG  argv contains 'controller_server', NOT our CFG"
echo "pos pid=$POS  argv: $(tr '\0' ' ' < /proc/$POS/cmdline 2>/dev/null | head -c 100)"
sleep 1

bash "$RUNNER" 1 > /tmp/cleanup_check_gate.log 2>&1
GATE_RC=$?
echo "gate rc=$GATE_RC"
grep -E '\[B6B stage1 lifecycle\]' /tmp/cleanup_check_gate.log | tail -1 || echo "(no smoke stage1 line)"
sleep 2

neg_alive=no; pos_alive=no
kill -0 "$NEG" 2>/dev/null && neg_alive=yes
kill -0 "$POS" 2>/dev/null && pos_alive=yes
echo "NEG alive=$neg_alive (want yes: unrelated spared)"
echo "POS alive=$pos_alive (want no: genuine CFG target killed)"

ok=1
[ "$GATE_RC" = 0 ] || { echo "FAIL: gate runner exited $GATE_RC"; ok=0; }
[ "$neg_alive" = yes ] || { echo "FAIL: unrelated proc was killed (over-broad match)"; ok=0; }
[ "$pos_alive" = no ]  || { echo "FAIL: genuine CFG target survived (cleanup no-op / pid-extraction broken)"; ok=0; }
kill -9 "$NEG" "$POS" 2>/dev/null || true

if [ "$ok" = 1 ]; then
  echo "CLEANUP CHECK: PASS (gate ok, spares unrelated, kills genuine)"
  exit 0
fi
echo "CLEANUP CHECK: FAIL"
exit 1
