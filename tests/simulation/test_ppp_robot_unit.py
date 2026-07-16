"""Open-loop PPP unit physics tests (SC-v12u / FR-SIM-11).

Optional foundation suite: not required on every PR gate.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt
import pytest

from fret.simulation.ppp_unit import PPPUnitRobot, mujoco_available

pytestmark = pytest.mark.skipif(
    not mujoco_available(),
    reason="mujoco package not installed",
)


def _robot() -> PPPUnitRobot:
    return PPPUnitRobot(
        initial_positions=np.array([1.0, 1.0, 1.5], dtype=np.float64)
    )


def _displace(
    robot: PPPUnitRobot,
    velocity: npt.NDArray[np.float64],
    *,
    seconds: float = 1.0,
) -> npt.NDArray[np.float64]:
    dt = 0.02
    steps = int(round(seconds / dt))
    for _ in range(steps):
        robot.step_velocity(velocity, dt=dt)
    return robot.get_positions()


@pytest.mark.parametrize(
    ("axis", "command", "expected_delta"),
    [
        (0, np.array([0.5, 0.0, 0.0]), 0.5),
        (1, np.array([0.0, 0.5, 0.0]), 0.5),
        (2, np.array([0.0, 0.0, 0.3]), 0.3),
        (0, np.array([-0.4, 0.0, 0.0]), -0.4),
        (1, np.array([0.0, -0.4, 0.0]), -0.4),
        (2, np.array([0.0, 0.0, -0.25]), -0.25),
    ],
)
def test_ppp_unit_axis_motion_within_tolerance(
    axis: int,
    command: npt.NDArray[np.float64],
    expected_delta: float,
) -> None:
    """When a single-axis velocity is commanded for 1 s, that axis moves ≈ v·t."""
    robot = _robot()
    q0 = robot.get_positions().copy()
    q1 = _displace(robot, command, seconds=1.0)
    delta = float(q1[axis] - q0[axis])
    assert delta == pytest.approx(expected_delta, rel=0.15, abs=0.05)
    for other in range(3):
        if other == axis:
            continue
        assert abs(float(q1[other] - q0[other])) < 0.05


def test_ppp_unit_zero_command_holds_altitude() -> None:
    """When ctrl=0 for 2 s, Z must not sag more than 2 cm (gravcomp)."""
    robot = _robot()
    robot.step_velocity(np.array([0.0, 0.0, 0.2]), dt=0.5)
    z0 = float(robot.get_positions()[2])
    for _ in range(100):
        robot.step_velocity(np.zeros(3), dt=0.02)
    z1 = float(robot.get_positions()[2])
    assert abs(z1 - z0) <= 0.02


def test_ppp_unit_mjcf_exposes_three_velocity_actuators() -> None:
    """Sandbox MJCF must bind three prismatic velocity actuators."""
    robot = _robot()
    assert robot.joint_names == ("joint_x", "joint_y", "joint_z")
    assert robot.mjcf_path.name == "ppp_unit.xml"
    assert robot.timestep_s == pytest.approx(0.002)
