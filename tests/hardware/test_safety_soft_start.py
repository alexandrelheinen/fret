"""FR-HW-44: soft start at measured pose; torque off on shutdown."""

from __future__ import annotations

import logging

import numpy as np
import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.safety import SafetyConfig, SafetySupervisor


def test_soft_start_goals_equal_measured_fr_hw_44() -> None:
    limits = np.array([[-2.0, 2.0]] * 6, dtype=np.float64)
    s = SafetySupervisor(limits, config=SafetyConfig(limit_margin_rad=0.0))
    measured = np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0], dtype=np.float64)
    goals = s.soft_start_goals(measured)
    assert np.allclose(goals, measured)


def test_enable_writes_measured_before_torque_fr_hw_44() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    for sid in range(1, 7):
        bus.servo(sid).present_position = 2100
        bus.servo(sid)._sync_regs_out()
    core.cycle(0.0)
    assert all(bus.servo(sid).torque_enable == 0 for sid in range(1, 7))
    core.enable_torque(0.0)
    assert core.state is BridgeState.ENABLED
    assert all(bus.servo(sid).torque_enable == 1 for sid in range(1, 7))
    for sid in range(1, 7):
        assert bus.servo(sid).goal_position == bus.servo(sid).present_position


def test_shutdown_disables_torque_fr_hw_44(
    caplog: pytest.LogCaptureFixture,
) -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    with caplog.at_level(logging.WARNING):
        core.disconnect()
    assert all(bus.servo(sid).torque_enable == 0 for sid in range(1, 7))
    assert any("arm goes limp" in r.message for r in caplog.records)
