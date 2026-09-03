# Launch the standalone tracking chain (WSL2 / Gazebo Harmonic).
#
#   ros2 launch linear_mpc_controller linear_mpc_sim.launch.py \
#       world:=tracking_empty.sdf use_sim_time:=True
#
# NOTE: the Gazebo/TurtleBot3 bringup is environment-specific; this file
# wires the controller nodes and parameters.  Full TurtleBot3 integration is
# the U5/C3 acceptance item (not yet exercised on this machine).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SHARE = get_package_share_directory("linear_mpc_controller")
WORLDS_DIR = os.path.join(PKG_SHARE, "worlds")
CONFIG_DIR = os.path.join(PKG_SHARE, "config")


def _setup(context, *args, **kwargs):
    world_name = LaunchConfiguration("world").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    world_path = os.path.join(WORLDS_DIR, os.path.basename(world_name))
    if not os.path.isfile(world_path):
        raise RuntimeError(f"world not found in {WORLDS_DIR}: {world_name}")

    sim = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_path], output="screen")
    mpc = Node(
        package="linear_mpc_controller",
        executable="linear_mpc_node",
        name="linear_mpc_node",
        output="screen",
        parameters=[
            os.path.join(CONFIG_DIR, "linear_mpc_params.yaml"),
            {"use_sim_time": use_sim_time.lower() in ("1", "true")},
        ],
        remappings=[],
    )
    arbiter = Node(
        package="linear_mpc_controller",
        executable="velocity_arbiter_node",
        name="velocity_arbiter",
        output="screen",
        parameters=[
            os.path.join(CONFIG_DIR, "velocity_arbiter.yaml"),
            {"use_sim_time": use_sim_time.lower() in ("1", "true")},
        ],
    )
    return [sim, mpc, arbiter]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="tracking_empty.sdf"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
