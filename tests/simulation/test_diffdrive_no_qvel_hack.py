"""Guard: unit diff-drive paths must not use the race qvel projection."""

from __future__ import annotations

import fret.control.dubins_wheel_model as race_hack
from fret.simulation import diffdrive_unit, turtlebot3_unit
from fret.simulation.diffdrive_unit import DiffDriveUnitRobot
from fret.simulation.turtlebot3_unit import TurtleBot3UnitRobot


def test_diffdrive_unit_declares_no_qvel_hack() -> None:
    """Policy flag must stay False (FR-SIM-11)."""
    assert DiffDriveUnitRobot.USES_NONHOLONOMIC_QVEL_HACK is False


def test_turtlebot3_unit_declares_no_qvel_hack() -> None:
    """TB3 unit policy flag must stay False (FR-SIM-11)."""
    assert TurtleBot3UnitRobot.USES_NONHOLONOMIC_QVEL_HACK is False


def test_unit_modules_do_not_bind_race_hack() -> None:
    """Unit sandboxes must not import or bind the race projection helper."""
    hack = race_hack.enforce_slide_yaw_nonholonomic_qvel
    for module in (diffdrive_unit, turtlebot3_unit):
        assert "dubins_wheel_model" not in module.__dict__
        assert hack.__name__ not in module.__dict__
        assert hack not in vars(module).values()
