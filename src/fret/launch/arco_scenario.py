"""Launch: Full ARCO obstacle-avoidance demonstration scenario.

Starts the complete FRET pipeline against the ``arco_scenario.sdf`` world
(3 static box obstacles).  Equivalent to running::

    ros2 launch fret sitl.py scenario:=obstacle_avoidance

Usage::

    ros2 launch fret arco_scenario.py
    ros2 launch fret arco_scenario.py model:=scara

Arguments:
    model (str, default: scara)
        Robot model name.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the ARCO obstacle-avoidance scenario."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="scara",
        description="Robot model name",
    )

    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "sitl.py"])
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "scenario": "obstacle_avoidance",
        }.items(),
    )

    return LaunchDescription(
        [
            model_arg,
            sitl_launch,
        ]
    )
