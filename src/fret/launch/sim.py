"""Unified FRET simulation launcher with model selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "empty.sdf"],
        output="screen",
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

    return [
        robot_state_publisher,
        gz_sim,
        spawn_robot,
    ]


def generate_launch_description():
    """Generate the simulation launch description."""
    return LaunchDescription(
        [
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
