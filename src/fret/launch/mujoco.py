"""Launch: MuJoCo simulator bridge for FRET SITL.

Usage::

    ros2 launch fret mujoco.py model:=dubins scenario:=dubins_race

Arguments:
    model (str, default: dubins)
        Robot model name.
    scenario (str, default: dubins_race)
        Scenario stem used for MJCF resolution.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the MuJoCo bridge node."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="dubins",
        description="Robot model name",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="dubins_race",
        description="Scenario stem for MJCF resolution",
    )
    physics_mode_arg = DeclareLaunchArgument(
        "physics_mode",
        default_value="false",
        description="Enable MuJoCo physics SITL (mj_step + actuators)",
    )

    mujoco_config = PathJoinSubstitution(
        [pkg_share, "config", "simulation", "mujoco.yml"]
    )

    mujoco_bridge_node = Node(
        package="fret",
        executable="mujoco_bridge",
        name="mujoco_bridge_node",
        output="screen",
        parameters=[
            mujoco_config,
            {
                "model": LaunchConfiguration("model"),
                "scenario": LaunchConfiguration("scenario"),
                "physics_mode": LaunchConfiguration("physics_mode"),
            },
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            scenario_arg,
            physics_mode_arg,
            mujoco_bridge_node,
        ]
    )
