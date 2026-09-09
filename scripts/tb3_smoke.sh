#!/bin/bash
# TB3 Gazebo closed-loop smoke (P0): headless Gazebo + TurtleBot3 +
# trajectory_server(circle) + linear_mpc_node + velocity_arbiter.
# Runs ~3 min, then tears everything down.  Output: /tmp/tb3_smoke.json
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export HOME=$HOME

# teardown: kill everything the smoke started
cleanup() {
  pkill -f "tracking_gz.launch" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "trajectory_server" 2>/dev/null
  pkill -f "linear_mpc_node" 2>/dev/null
  pkill -f "velocity_arbiter" 2>/dev/null
  pkill -f "ros_gz_bridge" 2>/dev/null
  pkill -f "parameter_bridge" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  pkill -f "spawn_tb3\|create" 2>/dev/null
  sleep 2
}
cleanup

echo "[$(date +%T)] launching tracking_gz (headless gz -s)..."
timeout 260 ros2 launch linear_mpc_controller tracking_gz.launch.py \
  track:=circle use_sim_time:=true > /tmp/tb3_smoke_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 50
echo "[$(date +%T)] checking..."
python3 "$HOME/ros2_ws/src/linear_mpc_controller/scripts/tb3_smoke_check.py" \
  /tmp/tb3_smoke.json 90
RC=$?
cleanup
kill $LAUNCH_PID 2>/dev/null
echo "smoke rc=$RC"
exit $RC
