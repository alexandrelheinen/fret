"""Guard: unit diff-drive path must not use the race qvel projection."""

from __future__ import annotations

import fret.control.dubins_wheel_model as race_hack
from fret.simulation import diffdrive_unit
from fret.simulation.diffdrive_unit import DiffDriveUnitRobot


def test_diffdrive_unit_declares_no_qvel_hack() -> None:
    """Policy flag must stay False (FR-SIM-11)."""
    assert DiffDriveUnitRobot.USES_NONHOLONOMIC_QVEL_HACK is False


def test_diffdrive_unit_module_does_not_bind_race_hack() -> None:
    """Unit sandbox must not import or bind the race projection helper."""
    assert "dubins_wheel_model" not in diffdrive_unit.__dict__
    hack_name = race_hack.enforce_slide_yaw_nonholonomic_qvel.__name__
    assert hack_name not in diffdrive_unit.__dict__
    assert (
        race_hack.enforce_slide_yaw_nonholonomic_qvel
        not in vars(diffdrive_unit).values()
    )
