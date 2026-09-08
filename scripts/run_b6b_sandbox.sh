#!/usr/bin/env bash
# B6B two-stage sandbox gate (spec part 6).  Exit code = verdict.
#   stage 1 : lifecycle configure/activate loads the plugin (pluginlib)
#   stage 2a: straight path, start on path
#   stage 2b: quarter-circle arc, OFFLINE start (steering must be excited)
set -u
WS=/home/zhouyi/lmpc_ws
REPO=$WS/src/linear_mpc_controller
CFG=$REPO/config/controller_server.yaml
LOG=/tmp/b6b_controller.log

full_cleanup() {
  pkill -9 -f "controller_server" 2>/dev/null || true
  pkill -9 -f "nav2_sandbox_robot" 2>/dev/null || true
  pkill -9 -f "gz[ ]sim" 2>/dev/null || true
  pkill -9 -f "ros_gz" 2>/dev/null || true
  pkill -9 -f "nav2_" 2>/dev/null || true
  pkill -9 -f "lifecycle_manager" 2>/dev/null || true
  pkill -9 -f "ros2 daemon" 2>/dev/null || true
  sleep 2
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
  if [ "$stage" = "1" ]; then
    # stage 1 drives the lifecycle itself (pluginlib load on configure)
    sleep 3
  else
    # stages 2a/2b: bring the server active, then wait for the action server
    ros2 lifecycle set /controller_server configure >/dev/null 2>&1 || true
    sleep 2
    ros2 lifecycle set /controller_server activate >/dev/null 2>&1 || true
  fi
  # wait for the FollowPath action server (stage 1 creates it inside the
  # smoke, so only 2a/2b wait here)
  local up=0
  if [ "$stage" != "1" ]; then
    for i in $(seq 1 60); do
      if ros2 action list 2>/dev/null | grep -q follow_path; then
        up=1; break
      fi
      sleep 1
    done
  else
    up=1
  fi
  if [ "$up" != 1 ]; then
    echo "B6B stage $stage: controller_server FollowPath action not up"
    tail -20 "$LOG"
    kill -9 $ROBOT_PID $SERVER_PID 2>/dev/null || true
    return 1
  fi
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
