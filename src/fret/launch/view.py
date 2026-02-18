"""Unified FRET visualization launcher with model selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fret.launch.model import resolve_robot_model

logger = logging.get_logger("fret")


def _launch_selected_model(context):
    model = LaunchConfiguration("model").perform(context)
    if model is None:
        raise ValueError("Missing 'model' launch argument.")

    # Resolve robot model from all fallback sources
    fret_share = get_package_share_directory("fret")
    robot_description = resolve_robot_model(model, fret_share)

    # RViz configuration paths
    rviz_path = os.path.join(fret_share, "rviz", f"{model}.rviz")
    default_rviz = os.path.join(fret_share, "rviz", "default.rviz")

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Use model-specific rviz config if present, else default
    rviz_config_file = rviz_path if os.path.exists(rviz_path) else default_rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config_file)],
    )

    return [
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ]


def generate_launch_description():
    """Generate the visualization launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="ur3",
                description=(
                    "Robot model to visualize " "(ur3, ur3e, ur5e, scara)."
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
