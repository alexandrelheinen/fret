"""Launch: full SITL pipeline (sim + planning + control + scene).

This is the main entry point for ``fretsim``.

Usage::

    ros2 launch fret sitl.py model:=scara scenario:=static_reach

Arguments:
    model (str, default: scara)
        Robot model name.
    scenario (str, default: static_reach)
        Scenario YAML stem (e.g. ``static_reach``).  Must match a file under
        ``fret/config/scenarios/<scenario>.yml``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the SITL pipeline."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="scara",
        description="Robot model name",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="static_reach",
        description="Scenario YAML stem (config/scenarios/<scenario>.yml)",
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/sim.py"]),
        launch_arguments={
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    scenario_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "scenarios",
            [LaunchConfiguration("scenario"), ".yml"],
        ]
    )
    controller_config = PathJoinSubstitution(
        [pkg_share, "config", "controllers", "jacobian.yml"]
    )

    planner_node = Node(
        package="fret",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            scenario_config,
        ],
    )

    controller_node = Node(
        package="fret",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            controller_config,
        ],
    )

    scene_acquisition_node = Node(
        package="fret",
        executable="scene_acquisition_node",
        name="scene_acquisition_node",
        output="screen",
        parameters=[{"model": LaunchConfiguration("model")}],
    )

    return LaunchDescription(
        [
            model_arg,
            scenario_arg,
            sim_launch,
            scene_acquisition_node,
            planner_node,
            controller_node,
        ]
    )
