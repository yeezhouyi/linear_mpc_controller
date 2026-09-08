#!/usr/bin/env bash
# B6B two-stage sandbox gate (spec part 6).  Exit code = verdict.
#   stage 1 : lifecycle configure/activate loads the plugin (pluginlib)
#   stage 2a: straight path, start on path
#   stage 2b: quarter-circle arc, OFFLINE start (steering must be excited)
#
# Sequencing discipline (B6B review): the runner NEVER drives the stack
# through ros2 CLI (lifecycle/action/topic all sit on the daemon whose graph
# cache makes readiness answers stale).  It only starts processes; the smoke
# script does configure/activate/plugin-assert via typed in-process service
# clients and waits for the action server via rclpy ActionClient.
set -u
WS=/home/zhouyi/lmpc_ws
REPO=$WS/src/linear_mpc_controller
CFG=$REPO/config/controller_server.yaml
LOG=/tmp/b6b_controller.log

# full_cleanup kills ONLY this B6B stack, precisely (spec part 5, pitfall
# 5): a `pkill -9 -f <pattern>` matches the FULL command line, so it also
# hits the ancestor `bash -c "...pattern..."` that launched this script
# (self-kill that looks like an unexplained mid-gate death) and broad
# patterns like "nav2_" or "ros2 daemon" take down unrelated processes or
# the user's own daemon.  Instead:
#   * controller_server + its `ros2 run` launcher are identified by OUR
#     config path on their command line (--params-file $CFG);
#   * the sandbox robot is identified by its unique script name;
#   * processes on this shell's own ancestor chain are always skipped;
#   * the user's `ros2 daemon` is NEVER touched (no CLI is used anymore).
is_self_ancestor() {
  local p=$1 a=$self
  while [ "$a" != "0" ] && [ -n "$a" ]; do
    [ "$a" = "$p" ] && return 0
    a=$(ps -o ppid= -p "$a" 2>/dev/null | tr -d ' ')
  done
  return 1
}
full_cleanup() {
  local self=$$ p
  for p in $(ps -eo pid=,args= \
              | grep -F -e "$CFG" -e "nav2_sandbox_robot.py" \
              | awk '{print $1}' | sort -u); do
    [ "$p" = "$self" ] && continue
    is_self_ancestor "$p" && continue
    kill -9 "$p" 2>/dev/null || true
  done
  sleep 1
}

run_stage() {
  local stage=$1
  local robot_args=()
  if [ "$stage" = "2b" ]; then
    robot_args=(--ros-args -p start_x:=-0.10 -p start_y:=-0.30 -p start_yaw:=0.35)
  fi
  echo "##### B6B stage $stage: cleanup #####"
  full_cleanup
  echo "##### B6B stage $stage: launch sandbox robot #####"
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS/install/setup.bash"
  set -u
  python3 "$REPO/scripts/nav2_sandbox_robot.py" "${robot_args[@]}" \
    > /tmp/b6b_robot.log 2>&1 &
  ROBOT_PID=$!
  sleep 2
  echo "##### B6B stage $stage: launch controller_server #####"
  ros2 run nav2_controller controller_server \
    --ros-args --params-file "$CFG" > "$LOG" 2>&1 &
  SERVER_PID=$!
  # Give the server a moment to spin its lifecycle + action services up;
  # the smoke script then waits in-process (ChangeState/GetState services,
  # ActionClient.wait_for_server) so no daemon-cached readiness is trusted.
  sleep 4
  echo "##### B6B stage $stage: smoke #####"
  python3 "$REPO/scripts/nav2_plugin_smoke.py" --stage "$stage"
  local rc=$?
  echo "##### B6B stage $stage: smoke exit $rc — teardown #####"
  kill -9 $ROBOT_PID $SERVER_PID 2>/dev/null || true
  full_cleanup
  return $rc
}

STAGES=("$@")
if [ ${#STAGES[@]} -eq 0 ]; then STAGES=(1 2a 2b); fi
overall=0
for st in "${STAGES[@]}"; do
  run_stage "$st" || overall=1
done
if [ "$overall" = 0 ]; then
  echo "B6B SANDBOX VERIFICATION: PASS (load + straight + steering)"
else
  echo "B6B SANDBOX VERIFICATION: FAIL"
fi
exit $overall
