"""Physics sandboxes and simulation helpers (unit robots, no ROS)."""

from fret.simulation.diffdrive_unit import (
    DiffDriveUnitRobot,
    diffdrive_unit_mjcf_path,
)
from fret.simulation.mujoco_camera import (
    PERCEPTION_CAMERA_ID,
    PERCEPTION_HEIGHT_PX,
    PERCEPTION_WIDTH_PX,
    MujocoCameraAdapter,
    MujocoCameraCapture,
    extrinsics_from_mujoco_camera,
    intrinsics_from_fovy,
    mujoco_xmat_to_opencv_rotation,
    project_world_point,
)
from fret.simulation.mujoco_util import mujoco_available
from fret.simulation.turtlebot3_unit import (
    TurtleBot3UnitRobot,
    body_velocity_to_wheel_rates,
    clip_wheel_rates,
    turtlebot3_unit_mjcf_path,
)

__all__ = [
    "PERCEPTION_CAMERA_ID",
    "PERCEPTION_HEIGHT_PX",
    "PERCEPTION_WIDTH_PX",
    "DiffDriveUnitRobot",
    "MujocoCameraAdapter",
    "MujocoCameraCapture",
    "TurtleBot3UnitRobot",
    "body_velocity_to_wheel_rates",
    "clip_wheel_rates",
    "diffdrive_unit_mjcf_path",
    "extrinsics_from_mujoco_camera",
    "intrinsics_from_fovy",
    "mujoco_available",
    "mujoco_xmat_to_opencv_rotation",
    "project_world_point",
    "turtlebot3_unit_mjcf_path",
]
