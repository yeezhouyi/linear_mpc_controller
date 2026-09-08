#!/usr/bin/env bash
# B6B negative control runner (spec discipline):
#   * unit-quaternion END pose (in fact every pose) -> goal yaw unsatisfiable
#     -> the goal can never self-succeed, so ONLY the terminal-stop clamp can
#     stop the robot driving past the plan end (a tangent-yaw path would let
#     the goal checker succeed first: a false pass);
#   * nc_patch_b6b.py disables exactly the normal-branch terminal clamp
#     (single variable, same binary otherwise);
#   * verdict: max_disp(patched) - max_disp(intact) >= 2.0 m proves the clamp
#     is what stops the overshoot.  Also enforces a harness precondition:
#     the intact run must stop near the arc end (2.356 m + margin), i.e. it
#     must NOT itself be a runaway.
set -u
WS=/home/zhouyi/lmpc_ws
REPO=$WS/src/linear_mpc_controller
PATCH=$REPO/scripts/nc_patch_b6b.py
RUN=$REPO/scripts/run_b6b_sandbox.sh
ARC_LEN=$(python3 -c "import math; print(round(math.pi*1.5/2,3))")   # 2.356
DIFF_MIN=2.0

build() {
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  cd "$WS" || exit 1
  colcon build --packages-select linear_mpc_controller \
    --cmake-args -DLINEAR_MPC_WITH_OSQP=ON -DBUILD_TESTING=OFF \
    > /tmp/nc_build.log 2>&1 || { echo "BUILD FAILED"; tail -15 /tmp/nc_build.log; exit 1; }
}

run_nc() {
  local tag=$1
  bash "$RUN" nc > "/tmp/nc_run_${tag}.log" 2>&1
  local rc=$?
  if [ "$rc" != 0 ]; then
    echo "NC scenario run failed (rc=$rc); tail:"
    tail -12 "/tmp/nc_run_${tag}.log"
    return 1
  fi
  grep -oE 'NC_MAXDISP=[0-9.]+' "/tmp/nc_run_${tag}.log" \
    | tail -1 | cut -d= -f2
}

echo "===== B6B NC: terminal-stop clamp causal test ====="
echo "arc length = $ARC_LEN m ; required overshoot diff >= $DIFF_MIN m"

# guard: never start from a stray patched state
python3 "$PATCH" status > /dev/null 2>&1
if [ $? -eq 1 ]; then
  echo "stray patch state -- reverting and rebuilding first"
  python3 "$PATCH" revert
  build
fi

echo "----- Phase A: INTACT build -----"
A=$(run_nc intact) || exit 1
echo "intact max_disp = $A m"
echo "$A" | grep -qE '^[0-9]+\.?[0-9]*$' || { echo "BAD intact value: $A"; exit 1; }
# harness precondition: intact robot must STOP near the arc end (it is a
# real overshoot stopper, not a coincidence of the scenario)
A_LO=$(python3 -c "print(round($ARC_LEN - 0.6,3))")
A_HI=$(python3 -c "print(round($ARC_LEN + 1.6,3))")
if python3 -c "exit(0 if $A_LO <= $A <= $A_HI else 1)"; then
  echo "precondition OK: intact stops in [$A_LO, $A_HI] m"
else
  echo "PRECONDITION FAIL: intact max_disp=$A outside [$A_LO, $A_HI] -- harness broken"
  exit 2
fi

echo "----- Phase B: clamp DISABLED build -----"
python3 "$PATCH" apply || exit 1
build
B=$(run_nc patched) || { python3 "$PATCH" revert; build; exit 1; }
echo "patched max_disp = $B m"
echo "$B" | grep -qE '^[0-9]+\.?[0-9]*$' || { echo "BAD patched value: $B"; python3 "$PATCH" revert; build; exit 1; }
python3 "$PATCH" revert
build
echo "reverted to pristine build"

echo "----- verdict -----"
echo "overshoot delta = $(python3 -c "print(round($B - $A,3))") m"
if python3 -c "exit(0 if $B - $A >= $DIFF_MIN else 1)"; then
  echo "NC VERDICT: PASS -- terminal-stop clamp is causal (removing it adds >= ${DIFF_MIN} m of overshoot)"
  exit 0
else
  echo "NC VERDICT: FAIL -- clamp removal did not produce the expected overshoot"
  exit 1
fi
