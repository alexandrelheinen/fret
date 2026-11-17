"""Robot model registry: profiles, variants, and resolution (FR-SYS-06..09).

Per PR1 (v1.5.0), FRET resolves per-model behavior (kinematics, controller
construction, and — where migrated — transfer-planner strategy and
simulation I/O) through this package instead of hardcoded string-literal
dispatch. See ``docs/robot-model-agnostic.md`` for the durable design and
``docs/robots/README.md`` for how to add a model.
"""

from __future__ import annotations

from fret.robots.profile import RobotProfile, RobotProfileError
from fret.robots.registry import (
    RobotRegistrationError,
    RobotRegistry,
    RobotVariantNotRegisteredError,
    UnknownRobotModelError,
    get_default_registry,
    register_controller,
    register_kinematics,
    register_sim_io,
    register_transfer_planner,
    reset_default_registry,
)

__all__ = [
    "RobotProfile",
    "RobotProfileError",
    "RobotRegistrationError",
    "RobotRegistry",
    "RobotVariantNotRegisteredError",
    "UnknownRobotModelError",
    "get_default_registry",
    "register_controller",
    "register_kinematics",
    "register_sim_io",
    "register_transfer_planner",
    "reset_default_registry",
]
