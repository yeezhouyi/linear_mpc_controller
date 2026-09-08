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
# Usage: check_cleanup_precision.sh [runner]   (default = repo runner)
# Exit 0 = both assertions hold.  Run once against a no-op/buggy runner to
# see POS go red, once against the fixed runner to see it go green.
set -u
RUNNER=${1:-/home/zhouyi/ros2_ws/src/linear_mpc_controller/scripts/run_b6b_sandbox.sh}
CFG=/home/zhouyi/lmpc_ws/src/linear_mpc_controller/config/controller_server.yaml

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
python3 -c 'import time; time.sleep(90)' "$CFG" &
POS=$!
echo "neg pid=$NEG  argv contains 'controller_server', NOT our CFG"
echo "pos pid=$POS  argv: $(tr '\0' ' ' < /proc/$POS/cmdline 2>/dev/null | head -c 100)"
sleep 1

bash "$RUNNER" 1 > /tmp/cleanup_check_gate.log 2>&1
echo "gate: $(grep -E 'stage1 lifecycle' /tmp/cleanup_check_gate.log || echo 'NO GATE LINE')"
sleep 2

neg_alive=no; pos_alive=no
kill -0 "$NEG" 2>/dev/null && neg_alive=yes
kill -0 "$POS" 2>/dev/null && pos_alive=yes
echo "NEG alive=$neg_alive (want yes: unrelated spared)"
echo "POS alive=$pos_alive (want no: genuine CFG target killed)"

ok=1
[ "$neg_alive" = yes ] || { echo "FAIL: unrelated proc was killed (over-broad match)"; ok=0; }
[ "$pos_alive" = no ]  || { echo "FAIL: genuine CFG target survived (cleanup no-op / pid-extraction broken)"; ok=0; }
kill -9 "$NEG" "$POS" 2>/dev/null || true

if [ "$ok" = 1 ]; then
  echo "CLEANUP CHECK: PASS (spares unrelated, kills genuine)"
  exit 0
fi
echo "CLEANUP CHECK: FAIL"
exit 1
