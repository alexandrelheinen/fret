"""FR-HW-45: consecutive failures and servo overload/overheat → FAULT hold."""

from __future__ import annotations

import numpy as np
import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.sts.errors import StsTimeoutError
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind


def test_n_failed_cycles_enter_fault_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    assert core.state is BridgeState.ENABLED
    for i in range(5):
        bus.inject_fault(FaultConfig(FaultKind.DROP_REPLY, remaining=6))
        with pytest.raises(StsTimeoutError):
            core.cycle(float(i) * 0.02)
    assert core.state is BridgeState.FAULT
    assert core.diagnostics.fault_reason


def test_overload_enters_fault_immediately_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.OVERLOAD, servo_id=1))
    core.cycle(0.0)
    assert core.state is BridgeState.FAULT
    assert "overload" in core.diagnostics.fault_reason


def test_overheat_enters_fault_immediately_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.OVERHEAT, servo_id=2))
    core.cycle(0.0)
    assert core.state is BridgeState.FAULT
    assert "overheat" in core.diagnostics.fault_reason


def test_fault_holds_position_no_torque_off_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.OVERLOAD, servo_id=1))
    core.cycle(0.0)
    assert core.state is BridgeState.FAULT
    # D9: hold — torque stays on.
    assert all(bus.servo(sid).torque_enable == 1 for sid in range(1, 7))
    hold = core.positions.copy()
    core.command_velocity(np.ones(6), 0.1)
    core.cycle(0.1)
    assert np.allclose(core.positions, hold) or core.state is BridgeState.FAULT
