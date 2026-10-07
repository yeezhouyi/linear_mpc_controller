#!/bin/bash
# TB3 smoke with lifecycle/topic diagnostics.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

cleanup() {
  pkill -f "tracking_gz.launch" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "trajectory_server" 2>/dev/null
  pkill -f "linear_mpc_node" 2>/dev/null
  pkill -f "velocity_arbiter" 2>/dev/null
  pkill -f "ros_gz_bridge" 2>/dev/null
  pkill -f "parameter_bridge" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  sleep 2
}
cleanup

echo "[$(date +%T)] launch"
timeout 280 ros2 launch linear_mpc_controller tracking_gz.launch.py \
  track:=circle use_sim_time:=true > /tmp/tb3_diag_launch.log 2>&1 &
sleep 45
echo "[$(date +%T)] lifecycle state:"
timeout 10 ros2 lifecycle get /linear_mpc_node 2>&1 || true
echo "[$(date +%T)] topic hz /linear_mpc_node/reference (3s):"
timeout 8 ros2 topic hz /linear_mpc_node/reference 2>&1 | tail -1 || true
echo "[$(date +%T)] topic hz /cmd_vel_mpc (3s):"
timeout 8 ros2 topic hz /cmd_vel_mpc 2>&1 | tail -1 || true
echo "[$(date +%T)] topic hz /cmd_vel (3s):"
timeout 8 ros2 topic hz /cmd_vel 2>&1 | tail -1 || true
echo "[$(date +%T)] node list:"
timeout 10 ros2 node list 2>&1 | head -12 || true
state=$(timeout 10 ros2 lifecycle get /linear_mpc_node 2>&1)
echo "state again: $state"
if echo "$state" | grep -q "inactive"; then
  echo "[$(date +%T)] manually activating..."
  timeout 10 ros2 lifecycle set /linear_mpc_node activate 2>&1 || true
  sleep 5
fi
echo "[$(date +%T)] odom check 60s:"
timeout 90 python3 "$HOME/ros2_ws/src/linear_mpc_controller/scripts/tb3_smoke_check.py" \
  /tmp/tb3_smoke.json 60 2>&1 | tail -2
cleanup
echo "diag done"
