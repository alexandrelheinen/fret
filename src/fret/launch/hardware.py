"""Launch: SO-101 hardware bridge + controller (FR-SYS-17).

Against a pty fake device (``fret-hw fake-bus``) the bridge reaches
``CONNECTED_DISABLED`` and publishes ``/joint_states`` at 50 Hz.
Torque enable on a real port requires an active ``SafetySupervisor``
(FR-SYS-19).
"""

from __future__ import annotations

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fret.resources import resolve


def _launch_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    robot = LaunchConfiguration("robot").perform(context)
    if robot not in {"so101", "so-101", "so_arm101"}:
        raise RuntimeError(
            f"hardware.py currently supports robot:=so101, got {robot!r}"
        )
    port = LaunchConfiguration("port").perform(context)
    calibration = LaunchConfiguration("calibration").perform(context)
    baud = LaunchConfiguration("baud_rate").perform(context)
    if not calibration:
        calibration = str(
            resolve("config", "hardware/so101_example_calibration.yaml")
        )
    controller_cfg = str(resolve("config", "controllers/so101.yml"))

    bridge = Node(
        package="fret",
        executable="bridge_node",
        name="bridge_node",
        output="screen",
        arguments=[
            "--port",
            port,
            "--baud-rate",
            baud,
            "--calibration",
            calibration,
        ],
    )
    controller = Node(
        package="fret",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {
                "model": "so101",
                "config_path": controller_cfg,
            }
        ],
    )
    return [bridge, controller]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="so101"),
            DeclareLaunchArgument("port", default_value="mock:so101"),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument("calibration", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
