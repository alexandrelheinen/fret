"""OpenMANIPULATOR-Y (OMY, 6-DOF) variant registration (FR-SYS-08).

MuJoCo kinematics stay behind a lazy import so the Pi hardware import
graph (FR-HW-61) never loads ``mujoco``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from fret.robots.registry import register_controller, register_kinematics

if TYPE_CHECKING:
    from fret.control.kinematics_open_manipulator_y import (
        OpenManipulatorYKinematics,
    )


@register_kinematics("open_manipulator_y")
def _build_omy_kinematics() -> OpenManipulatorYKinematics:
    from fret.control.kinematics_open_manipulator_y import (
        OpenManipulatorYKinematics,
    )

    return OpenManipulatorYKinematics()


@register_controller("open_manipulator_y")
def _build_omy_controller(model: str, config_path: str) -> object:
    from fret.control.controller_node import ControllerNode

    return ControllerNode(model=model, config_path=config_path, dof=6)
