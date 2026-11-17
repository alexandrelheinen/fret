"""OpenMANIPULATOR-X variant registration (FR-SYS-08).

MuJoCo kinematics stay behind a lazy import so the Pi hardware import
graph (FR-HW-61) never loads ``mujoco``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from fret.robots.registry import register_controller, register_kinematics

if TYPE_CHECKING:
    from fret.control.kinematics_open_manipulator_x import (
        OpenManipulatorXKinematics,
    )


@register_kinematics("open_manipulator_x")
def _build_omx_kinematics() -> OpenManipulatorXKinematics:
    from fret.control.kinematics_open_manipulator_x import (
        OpenManipulatorXKinematics,
    )

    return OpenManipulatorXKinematics()


@register_controller("open_manipulator_x")
def _build_omx_controller(model: str, config_path: str) -> object:
    from fret.control.controller_node import ControllerNode

    return ControllerNode(model=model, config_path=config_path, dof=4)
