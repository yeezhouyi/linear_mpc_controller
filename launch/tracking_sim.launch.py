# Standalone tracking closed loop (U5/C3, KTD11 standalone_tracking mode).
#
#   ros2 launch linear_mpc_controller tracking_sim.launch.py track:=circle
#
# Chain (single /cmd_vel writer contract):
#   tb3_simulation_launch.py  (Gazebo + TurtleBot3 + SLAM + idle Nav2)
#   trajectory_server  --nav_msgs/Path-->  linear_mpc_node  --cmd_vel_mpc-->
#   velocity_arbiter  --/cmd_vel (TwistStamped)-->
#   ros_gz_bridge (TwistStamped -> gz.msgs.Twist)  -->  diff_drive plugin
#
# Nav2 is brought up but idle (no goals); the arbiter guarantees only one
# candidate ever reaches /cmd_vel.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

PKG_SHARE = get_package_share_directory("linear_mpc_controller")
CONFIG_DIR = os.path.join(PKG_SHARE, "config")


def _setup(context, *args, **kwargs):
    track = LaunchConfiguration("track").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sim_time = use_sim_time.lower() in ("1", "true")
    # nav2_bringup evaluates some args with PythonExpression, which needs
    # Python literals ("True"), not YAML-style "true".
    headless_py = "True" if headless.lower() in ("1", "true") else "False"

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "tb3_simulation_launch.py",
            )
        ),
        launch_arguments={
            "slam": "True",
            "use_sim_time": "True",
            "autostart": "True",
            "headless": headless_py,
            "use_rviz": "False",
            "use_composition": "False",
            "world": os.path.join(PKG_SHARE, "worlds", "tracking_empty.sdf"),
        }.items(),
    )

    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_ts_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist"],
        output="screen",
    )

    traj_server = Node(
        package="linear_mpc_controller",
        executable="trajectory_server.py",
        name="trajectory_server",
        output="screen",
        parameters=[
            {"track": track, "rate_hz": 2.0,
             "topic": "/linear_mpc_node/reference",
             "use_sim_time": sim_time},
        ],
    )

    mpc = Node(
        package="linear_mpc_controller",
        executable="linear_mpc_node",
        name="linear_mpc_node",
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

    # Autostart the lifecycle MPC node: unconfigured -> configure -> activate
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
    return [tb3_sim, cmd_vel_bridge, traj_server, mpc, arbiter,
            activate_on_inactive, configure]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("track", default_value="circle"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
