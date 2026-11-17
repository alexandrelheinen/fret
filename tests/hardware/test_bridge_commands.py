"""FR-HW-22 / FR-HW-23: velocity commands via VelocityToPositionIntegrator."""

from __future__ import annotations

import numpy as np
import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock


def test_velocity_command_integrates_and_sync_writes_fr_hw_22_23() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    assert core.state is BridgeState.CONNECTED_DISABLED

    v = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    t = 0.0
    core.enable_torque(t)
    assert core.state is BridgeState.ENABLED
    for i in range(10):
        t = (i + 1) * 0.02
        core.command_velocity(v, t)
        core.renew_enable_lease(t)
        core.cycle(t)

    # Goal on servo 1 should have moved away from mid toward positive.
    assert bus.servo(1).goal_position != 2048


def test_nan_command_rejected_and_counted() -> None:
    core, _ = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque()
    bad = np.array([np.nan, 0, 0, 0, 0, 0], dtype=np.float64)
    core.command_velocity(bad, 0.0)
    assert core.diagnostics.bad_commands == 1


def test_wrong_dof_command_rejected() -> None:
    core, _ = make_bridge_core_for_mock()
    core.connect()
    core.command_velocity(np.zeros(3), 0.0)
    assert core.diagnostics.bad_commands == 1
