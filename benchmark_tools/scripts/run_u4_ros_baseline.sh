#!/bin/bash
# U4 (reduced first pass): pure-MPC 4 tracks x 5 seeds in Gazebo.
# Per cell: boot sim, run motion gate (displacement), count MPC diagnostics
# messages, archive boot log.  Full metrics schema (e_y RMS vs reference)
# is delegated to the offline replayer on the same tracks (ref_core
# baseline); this pass establishes the ROS-layer completion matrix.
set -o pipefail
MPC=/home/zhouyi/ros2_ws/src/linear_mpc_controller
OUT=/home/zhouyi/ros2_ws/src/linear_mpc_controller/results/mpc_baseline_ros
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
cd "$MPC"
source ~/build_lmpc/install/setup.bash 2>/dev/null || source ~/install_lmpc/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=88
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/zhouyi/ros2_tunnel_explorer/install/tunnel_explorer_bringup/share/tunnel_explorer_bringup/config/fastdds_udp_only.xml

pkill -9 -f "gz [s]im" 2>/dev/null; pkill -9 -f "linear_mpc_[n]ode" 2>/dev/null
pkill -9 -f "trajectory_[s]erver" 2>/dev/null; pkill -9 -f "velocity_[a]rbiter" 2>/dev/null
pkill -9 -f "cmd_vel_ts_[b]ridge" 2>/dev/null
sleep 2
ros2 daemon stop >/dev/null 2>&1
rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null

n=0
for track in straight circle s_curve u_turn; do
  for seed in 0 1 2 3 4; do
    n=$((n+1))
    cell="$OUT/${track}_seed${seed}"
    mkdir -p "$cell"
    echo "[$n/20] $track seed=$seed"
    pkill -9 -f "gz [s]im" 2>/dev/null
    pkill -9 -f "linear_mpc_[n]ode" 2>/dev/null
    pkill -9 -f "trajectory_[s]erver" 2>/dev/null
    pkill -9 -f "velocity_[a]rbiter" 2>/dev/null
    pkill -9 -f "cmd_vel_ts_[b]ridge" 2>/dev/null
    sleep 2
    ros2 daemon stop >/dev/null 2>&1
    rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null

    setsid nohup ros2 launch linear_mpc_controller tracking_gz.launch.py \
      track:="$track" headless:=true use_sim_time:=true \
      > "$cell/launch.log" 2>&1 < /dev/null &
    sleep 80

    timeout 30 python3 $MPC/scripts/check_motion.py --topic /odom \
      --seconds 12 --min-displacement 0.05 > "$cell/motion.txt" 2>&1
    RC=$?
    echo "motion_rc=$RC" >> "$cell/motion.txt"

    timeout 10 ros2 topic echo /linear_mpc_node/diagnostics --once \
      > "$cell/diagnostics.txt" 2>&1 || true
    echo "cell rc=$RC" > "$cell/status.txt"
  done
done

echo "=== aggregate ==="
python3 - << 'PYEOF'
import json
import re
from pathlib import Path
out = Path("/home/zhouyi/ros2_ws/src/linear_mpc_controller/results/mpc_baseline_ros")
rows = []
for cell in sorted(out.iterdir()):
    if not cell.is_dir():
        continue
    mt = (cell / "motion.txt").read_text() if (cell / "motion.txt").exists() else ""
    passed = "PASS" in mt
    m = re.search(r"displacement=([0-9.]+)", mt)
    disp = float(m.group(1)) if m else 0.0
    rows.append({"cell": cell.name, "motion_pass": passed,
                 "displacement_m": disp})
ok = sum(1 for r in rows if r["motion_pass"])
summary = {"total": len(rows), "expected": 20, "motion_pass": ok, "cells": rows}
(out / "aggregate.json").write_text(json.dumps(summary, indent=1))
print(json.dumps({"total": len(rows), "motion_pass": ok}, indent=1))
PYEOF
echo "=== U4 DONE ==="
