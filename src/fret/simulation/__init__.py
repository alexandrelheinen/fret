"""Physics sandboxes and simulation helpers (unit robots, no ROS)."""

from fret.simulation.diffdrive_unit import (
    DiffDriveUnitRobot,
    body_velocity_to_wheel_rates,
    diffdrive_unit_mjcf_path,
)
from fret.simulation.mujoco_util import mujoco_available

__all__ = [
    "DiffDriveUnitRobot",
    "body_velocity_to_wheel_rates",
    "diffdrive_unit_mjcf_path",
    "mujoco_available",
]
