"""Launch: hardware-in-the-loop pipeline (bridge + controller).

Usage::

    ros2 launch fret hardware.py model:=open_manipulator_x port:=/dev/ttyUSB0

Arguments:
    model (str, default: open_manipulator_x)
        Robot model name.
    port (str, default: /dev/ttyUSB0)
        Serial port connected to the Arduino hardware interface.
    config (str, default: open_manipulator_x.yml)
        Controller YAML config file stem under ``config/controllers/``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the hardware-in-the-loop pipeline."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="open_manipulator_x",
        description="Robot model name",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the Arduino hardware bridge",
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="open_manipulator_x",
        description="Controller config YAML stem (config/controllers/<config>.yml)",
    )

    controller_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "controllers",
            [LaunchConfiguration("config"), ".yml"],
        ]
    )

    bridge_node = Node(
        package="fret",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baud_rate": 115200},
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

    return LaunchDescription(
        [
            model_arg,
            port_arg,
            config_arg,
            bridge_node,
            controller_node,
        ]
    )
