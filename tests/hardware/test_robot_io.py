"""Hardware RobotIO against the mock bus."""

from __future__ import annotations

import numpy as np

from fret.hardware.bridge_core import make_bridge_core_for_mock
from fret.hardware.robot_io import HardwareRobotIO


def test_hardware_robot_io_read_and_command() -> None:
    core, _ = make_bridge_core_for_mock()
    io = HardwareRobotIO(core)
    q = io.read_joint_state()
    assert q.shape == (6,)
    io.send_joint_velocity(np.zeros(6))
    io.set_gripper(0.5)
    assert io.now() >= 0.0
