"""Launch: visualise a robot model in RViz2.

Usage::

    ros2 launch fret view.py model:=scara

Arguments:
    model (str, default: scara)
        Robot model name.  Must match a XACRO file under
        ``fret/urdf/<model>.xacro``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the view configuration."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="scara",
        description="Robot model name (must match urdf/<model>.xacro)",
    )

    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", [LaunchConfiguration("model"), ".xacro"]]
    )
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", [LaunchConfiguration("model"), ".rviz"]]
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher,
            rviz2,
        ]
    )
