"""Launch UVC camera → ``sensor_msgs/Image`` + ``CameraInfo`` (FR-HW-30).

Uses the released Jazzy ``usb_cam`` package when available. Driver choice
is recorded in the PR5 evidence (``apt-cache policy`` on arm64).
"""

from __future__ import annotations

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fret.resources import resolve


def _launch_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    device = LaunchConfiguration("device").perform(context)
    config = LaunchConfiguration("config").perform(context)
    if not config:
        config = str(resolve("config", "hardware/camera.yml"))
    # usb_cam parameters file path; falls back to inline params if needed.
    node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="fret_usb_cam",
        output="screen",
        parameters=[
            {
                "video_device": device,
                "image_width": 1280,
                "image_height": 720,
                "framerate": 15.0,
                "pixel_format": "mjpeg2rgb",
                "camera_frame_id": "fret_cam0",
            }
        ],
        remappings=[
            ("image_raw", "/fret/camera/image_raw"),
            ("camera_info", "/fret/camera/camera_info"),
        ],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/fret-cam0"),
            DeclareLaunchArgument("config", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
