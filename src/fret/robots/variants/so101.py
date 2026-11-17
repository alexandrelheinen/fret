"""SO-101 variant registration (FR-SYS-08, PR2)."""

from __future__ import annotations

from fret.control.kinematics_so101 import SO101Kinematics
from fret.robots.registry import register_controller, register_kinematics


@register_kinematics("so101")
def _build_so101_kinematics() -> SO101Kinematics:
    return SO101Kinematics()


@register_controller("so101")
def _build_so101_controller(model: str, config_path: str) -> object:
    from fret.control.controller_node import ControllerNode

    return ControllerNode(model=model, config_path=config_path, dof=5)
