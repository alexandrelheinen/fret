"""Physics sandboxes and simulation helpers (unit robots, no ROS)."""

from fret.simulation.ppp_unit import (
    PPPUnitRobot,
    mujoco_available,
    ppp_unit_mjcf_path,
)

__all__ = [
    "PPPUnitRobot",
    "mujoco_available",
    "ppp_unit_mjcf_path",
]
