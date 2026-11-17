"""OpenMANIPULATOR-X variant registration (FR-SYS-08)."""

from __future__ import annotations

from fret.control.kinematics_open_manipulator_x import (
    OpenManipulatorXKinematics,
)
from fret.robots.registry import register_controller, register_kinematics


@register_kinematics("open_manipulator_x")
def _build_omx_kinematics() -> OpenManipulatorXKinematics:
    return OpenManipulatorXKinematics()


@register_controller("open_manipulator_x")
def _build_omx_controller(model: str, config_path: str) -> object:
    from fret.control.controller_node import ControllerNode

    return ControllerNode(model=model, config_path=config_path, dof=4)
