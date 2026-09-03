# Minimal standalone-tracking Gazebo bringup (U5/C3, KTD11).
#
#   ros2 launch linear_mpc_controller tracking_gz.launch.py
#
# No Nav2, no SLAM (KTD11 standalone_tracking): Gazebo + TurtleBot3 +
# bridges + trajectory_server + linear MPC + velocity arbiter.
#
# Sequencing: a setup process xacros the world and the robot SDF to /tmp;
# when it exits, gz sim + spawn + bridges + the MPC chain start.  The
# ros_gz_sim create node retries until the gz server answers.
#
# Single /cmd_vel writer: velocity_arbiter (TwistStamped).  The bridge
# maps ROS /cmd_vel (TwistStamped) -> gz.msgs.Twist for diff_drive.
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

PKG_SHARE = get_package_share_directory("linear_mpc_controller")
CONFIG_DIR = os.path.join(PKG_SHARE, "config")
TB3_SIM_SHARE = get_package_share_directory("nav2_minimal_tb3_sim")


def _setup(context, *args, **kwargs):
    track = LaunchConfiguration("track").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sim_time = use_sim_time.lower() in ("1", "true")

    world_src = os.path.join(PKG_SHARE, "worlds", "tracking_empty.sdf")
    robot_xacro = os.path.join(TB3_SIM_SHARE, "urdf", "gz_waffle.sdf.xacro")
    world_sdf = tempfile.mktemp(prefix="lmpc_world_", suffix=".sdf")
    robot_sdf = tempfile.mktemp(prefix="lmpc_robot_", suffix=".sdf")

    # 1) serialise both xacro passes in one setup process
    setup = ExecuteProcess(
        cmd=["bash", "-c",
             f"xacro -o '{world_sdf}' '{world_src}' && "
             f"xacro -o '{robot_sdf}' '{robot_xacro}'"],
        output="screen",
    )

    # 2) simulation + control chain start once the SDFs exist
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "2", world_sdf],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tb3",
        arguments=[
            "-world", "tracking_empty",
            "-file", robot_sdf,
            "-x", "0.0", "-y", "0.0", "-z", "0.1",
        ],
        output="screen",
    )
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_ts_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist"],
        output="screen",
    )
    bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tracking_bridges",
        parameters=[{
            "config_file": os.path.join(CONFIG_DIR, "tracking_bridge.yaml"),
            "expand_gz_topic_names": True,
            "use_sim_time": sim_time,
        }],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": open(
                os.path.join(TB3_SIM_SHARE, "urdf", "turtlebot3_waffle.urdf")
            ).read(),
            "use_sim_time": sim_time,
        }],
        output="screen",
    )
    traj_server = Node(
        package="linear_mpc_controller",
        executable="trajectory_server.py",
        name="trajectory_server",
        output="screen",
        parameters=[{
            "track": track, "rate_hz": 2.0,
            "topic": "/linear_mpc_node/reference",
            "use_sim_time": sim_time,
        }],
    )
    mpc = LifecycleNode(
        package="linear_mpc_controller",
        executable="linear_mpc_node",
        name="linear_mpc_node",
        namespace="",
        output="screen",
        parameters=[
            os.path.join(CONFIG_DIR, "linear_mpc_params.yaml"),
            {"use_sim_time": sim_time},
        ],
    )
    arbiter = Node(
        package="linear_mpc_controller",
        executable="velocity_arbiter_node",
        name="velocity_arbiter",
        output="screen",
        parameters=[
            os.path.join(CONFIG_DIR, "velocity_arbiter.yaml"),
            {"use_sim_time": sim_time},
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(mpc),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mpc,
            goal_state="inactive",
            entities=[EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(mpc),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )],
        )
    )

    chain = RegisterEventHandler(
        OnProcessExit(target_action=setup, on_exit=[
            gz_sim, spawn, bridges, cmd_vel_bridge, rsp, traj_server, mpc, arbiter,
            configure,
        ])
    )

    return [setup, chain, activate_on_inactive]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("track", default_value="circle"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
