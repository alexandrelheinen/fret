"""Unified FRET simulation launcher with model selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fret.launch.model import resolve_robot_model

logger = logging.get_logger("fret")


def _controller_parameters_for_model(model: str) -> dict:
    """Build controller node parameters that match the selected model."""
    if model.lower().startswith("ur"):
        return {
            "robot_model": model,
            "joint_states_topic": "/joint_states",
            "command_topic": "/joint_group_velocity_controller/commands",
            "base_frame": "base_link",
            "ee_frame": "tool0",
            "command_rate_hz": 50.0,
            "joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        }

    return {
        "robot_model": model,
        "joint_states_topic": "/joint_states",
        "command_topic": "/joint_group_velocity_controller/commands",
        "base_frame": "base_link",
        "ee_frame": "end_effector_link",
        "command_rate_hz": 50.0,
        "joint_names": [
            "joint_arm_0",
            "joint_arm_1",
            "joint_extension",
            "joint_tool_rotate",
        ],
    }


def _launch_selected_model(context):
    model = LaunchConfiguration("model").perform(context)
    if model is None:
        raise ValueError("Missing 'model' launch argument.")

    world = LaunchConfiguration("world").perform(context)

    # Resolve robot model from all fallback sources
    fret_share = get_package_share_directory("fret")
    robot_description = resolve_robot_model(model, fret_share)

    # Gazebo resolves model:// URIs by searching GZ_SIM_RESOURCE_PATH for a
    # directory whose name matches the first path component of the URI.
    # Meshes are installed to share/fret/meshes/, so Gazebo must find a
    # parent directory that contains a "fret/" subdirectory — i.e. share/.
    gz_resource_parent = os.path.dirname(fret_share)
    gz_env = os.environ.copy()
    existing = gz_env.get("GZ_SIM_RESOURCE_PATH", "")
    gz_env["GZ_SIM_RESOURCE_PATH"] = (
        f"{gz_resource_parent}:{existing}" if existing else gz_resource_parent
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_env["GZ_SIM_RESOURCE_PATH"]
        },
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            model,
            "-topic",
            "robot_description",
            "-z",
            "0.0",
        ],
    )

    # Bridge Gazebo joint states to a dedicated topic. joint_state_publisher
    # republishes a stable /joint_states stream from that source and also
    # ensures startup-time joint states exist even before Gazebo data arrives.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            f"/model/{model}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            (f"/model/{model}/joint_state", "/sim_joint_states"),
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "rate": 50.0,
                "source_list": ["/sim_joint_states"],
            }
        ],
    )

    controller_node = Node(
        package="fret",
        executable="controller",
        output="screen",
        parameters=[_controller_parameters_for_model(model)],
    )

    delayed_controller = TimerAction(period=2.0, actions=[controller_node])

    return [
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        gz_bridge,
        joint_state_publisher,
        delayed_controller,
    ]


def generate_launch_description():
    """Generate the simulation launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="empty.sdf",
                description=(
                    "Gazebo world file (absolute path or resource name). "
                    "Defaults to the built-in empty world."
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="ur3",
                description=(
                    "Robot model to simulate "
                    "(check ur_description for available options)."
                ),
            ),
            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
                description="Enable safety limits for UR models.",
            ),
            DeclareLaunchArgument(
                "safety_pos_margin",
                default_value="0.15",
                description="Safety margin used in UR xacro.",
            ),
            DeclareLaunchArgument(
                "safety_k_position",
                default_value="20",
                description="Safety controller gain used in UR xacro.",
            ),
            DeclareLaunchArgument(
                "tf_prefix",
                default_value='""',
                description="Joint name prefix used in UR xacro.",
            ),
            OpaqueFunction(function=_launch_selected_model),
        ]
    )
