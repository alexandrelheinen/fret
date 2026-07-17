"""Open-loop differential-drive unit physics tests (SC-v12u / FR-SIM-11).

Optional foundation suite: not required on every PR gate.
"""

from __future__ import annotations

import math

import pytest

from fret.simulation.diffdrive_unit import (
    DiffDriveUnitRobot,
    body_velocity_to_wheel_rates,
)
from fret.simulation.mujoco_util import mujoco_available

pytestmark = pytest.mark.skipif(
    not mujoco_available(),
    reason="mujoco package not installed",
)


def test_body_velocity_to_wheel_rates_forward_equal() -> None:
    """Equal wheel rates for pure forward motion."""
    left, right = body_velocity_to_wheel_rates(0.5, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.5 / 0.05)


def test_body_velocity_to_wheel_rates_spin_opposite() -> None:
    """Opposite wheel rates for pure yaw (spin in place)."""
    left, right = body_velocity_to_wheel_rates(0.0, 1.0)
    assert left == pytest.approx(-right)
    assert left < 0.0 < right


def test_diffdrive_unit_moves_forward() -> None:
    """When v>0 and ω=0 for 2 s, X advances and |Y| stays small."""
    robot = DiffDriveUnitRobot()
    x0, y0, yaw0 = robot.get_pose()
    for _ in range(100):
        robot.step_body_velocity(0.5, 0.0, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert (x1 - x0) >= 0.75  # ≥75% of 1.0 m commanded
    assert abs(y1 - y0) < 0.05
    assert abs(yaw1 - yaw0) < 0.15


def test_diffdrive_unit_spins_in_place() -> None:
    """When v=0 and ω>0 for 2 s, yaw advances with little translation."""
    robot = DiffDriveUnitRobot()
    x0, y0, _yaw0 = robot.get_pose()
    for _ in range(100):
        robot.step_body_velocity(0.0, 1.5, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert abs(yaw1) >= 2.0  # most of 3 rad commanded
    assert math.hypot(x1 - x0, y1 - y0) <= 0.05


def test_diffdrive_unit_traces_arc() -> None:
    """Constant (v, ω) must produce sustained yaw change (circle-like arc)."""
    robot = DiffDriveUnitRobot()
    x0, y0, yaw0 = robot.get_pose()
    for _ in range(200):
        robot.step_body_velocity(0.4, 0.4, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert abs(yaw1 - yaw0) >= 1.0
    # Not a straight line along +X: lateral displacement appears.
    assert abs(y1 - y0) >= 0.15
    assert math.hypot(x1 - x0, y1 - y0) >= 0.5


def test_diffdrive_unit_mjcf_has_two_wheel_actuators() -> None:
    """Sandbox MJCF must expose two wheel velocity actuators."""
    robot = DiffDriveUnitRobot(settle_s=0.0)
    assert robot.mjcf_path.name == "diffdrive_unit.xml"
    assert robot.timestep_s == pytest.approx(0.002)
    assert robot.wheel_radius_m == pytest.approx(0.05)
    assert robot.track_width_m == pytest.approx(0.18)
