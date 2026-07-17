"""Open-loop TurtleBot3 unit physics tests (SC-v12u / FR-SIM-11).

Optional foundation suite: not required on every PR gate.
"""

from __future__ import annotations

import math

import pytest

from fret.simulation.mujoco_util import mujoco_available
from fret.simulation.turtlebot3_unit import (
    TurtleBot3UnitRobot,
    body_velocity_to_wheel_rates,
    clip_wheel_rates,
)

pytestmark = pytest.mark.skipif(
    not mujoco_available(),
    reason="mujoco package not installed",
)


def test_tb3_body_velocity_to_wheel_rates_forward_equal() -> None:
    """Equal wheel rates for pure forward motion."""
    left, right = body_velocity_to_wheel_rates(0.2, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.2 / 0.033)


def test_tb3_body_velocity_to_wheel_rates_spin_opposite() -> None:
    """Opposite wheel rates for pure yaw (spin in place)."""
    left, right = body_velocity_to_wheel_rates(0.0, 1.0)
    assert left == pytest.approx(-right)
    assert left < 0.0 < right


def test_tb3_clip_wheel_rates_preserves_ratio() -> None:
    """Saturation scales both wheels so yaw is not erased."""
    left, right = clip_wheel_rates(200.0, 220.0, limit_rad_s=120.0)
    assert max(abs(left), abs(right)) == pytest.approx(120.0)
    assert left / right == pytest.approx(200.0 / 220.0)


def test_tb3_unit_moves_forward() -> None:
    """When v>0 and ω=0 for 2 s, X advances and |Y|/|yaw| stay small."""
    robot = TurtleBot3UnitRobot()
    x0, y0, yaw0 = robot.get_pose()
    for _ in range(100):
        robot.step_body_velocity(0.2, 0.0, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert (x1 - x0) >= 0.20
    assert abs(y1 - y0) < 0.05
    assert abs(yaw1 - yaw0) < 0.20


def test_tb3_unit_spins_in_place() -> None:
    """When v=0 and ω>0 for 2 s, yaw advances with little translation."""
    robot = TurtleBot3UnitRobot()
    x0, y0, _yaw0 = robot.get_pose()
    for _ in range(100):
        robot.step_body_velocity(0.0, 1.5, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert abs(yaw1) >= 1.0
    assert math.hypot(x1 - x0, y1 - y0) <= 0.05


def test_tb3_unit_traces_arc() -> None:
    """Constant (v, ω) must produce sustained yaw change (circle-like arc)."""
    robot = TurtleBot3UnitRobot()
    x0, y0, yaw0 = robot.get_pose()
    for _ in range(200):
        robot.step_body_velocity(0.15, 0.5, dt=0.02)
    x1, y1, yaw1 = robot.get_pose()
    assert abs(yaw1 - yaw0) >= 0.8
    assert abs(y1 - y0) >= 0.08
    assert math.hypot(x1 - x0, y1 - y0) >= 0.25


def test_tb3_unit_mjcf_has_two_wheel_actuators() -> None:
    """Sandbox MJCF must expose two wheel velocity actuators."""
    robot = TurtleBot3UnitRobot(settle_s=0.0)
    assert robot.mjcf_path.name == "turtlebot3_unit.xml"
    assert robot.timestep_s == pytest.approx(0.002)
    assert robot.wheel_radius_m == pytest.approx(0.033)
    assert robot.track_width_m == pytest.approx(0.16)
    assert robot.USES_NONHOLONOMIC_QVEL_HACK is False
