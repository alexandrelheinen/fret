"""FR-HW-46, FR-HW-47, FR-HW-70: clear/enable, lease, latched stop."""

from __future__ import annotations

import numpy as np
import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.sts.errors import StsError
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind


def test_controller_fault_holds_and_requires_clear_enable_fr_hw_46() -> None:
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    core.set_controller_fault(True)
    assert core.state is BridgeState.FAULT
    with pytest.raises(StsError, match="clear"):
        core.enable_torque(0.1)
    core.clear()
    assert core.state is BridgeState.CONNECTED_DISABLED
    core.enable_torque(0.2)
    assert core.state is BridgeState.ENABLED


def test_enable_rejected_while_fault_fr_hw_46() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.OVERLOAD, servo_id=1))
    core.cycle(0.0)
    assert core.state is BridgeState.FAULT
    with pytest.raises(StsError, match="clear"):
        core.enable_torque(0.1)


def test_enable_lease_expiry_holds_like_watchdog_fr_hw_47() -> None:
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    # Command + lease at t=0; lease default 0.5 s.
    core.command_velocity(np.zeros(6), 0.0)
    core.cycle(0.0)
    # Keep commands fresh but let lease expire.
    t = 0.0
    while t < 0.6:
        t += 0.02
        core.command_velocity(np.zeros(6), t)
        core.cycle(t)
    assert core.diagnostics.lease_expired is True
    assert core.state is BridgeState.ENABLED  # hold, not FAULT


def test_stop_latches_category_2_hold_fr_hw_70() -> None:
    from fret.hardware.safety import SafetySupervisor

    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    core.stop()
    assert core.diagnostics.stop_latched is True
    assert not hasattr(core, "emergency_stop")
    core.command_velocity(np.ones(6), 0.05)
    core.cycle(0.05)
    assert all(bus.servo(sid).torque_enable == 1 for sid in range(1, 7))
    core.clear()
    assert isinstance(core.safety, SafetySupervisor)
    assert core.safety.stop_latched is False
    core.renew_enable_lease(0.1)
    core.command_velocity(np.zeros(6), 0.1)
    core.cycle(0.1)


def test_watchdog_and_lease_both_expire_one_hold() -> None:
    from fret.hardware.safety import SafetyConfig, SafetySupervisor

    limits = np.array([[-2.0, 2.0]] * 6, dtype=np.float64)
    s = SafetySupervisor(
        limits,
        config=SafetyConfig(
            watchdog_s=0.1,
            enable_lease_s=0.1,
            limit_margin_rad=0.0,
            max_joint_velocity=10.0,
        ),
    )
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6)
    goals = np.ones(6)
    out = s.filter_goals(goals, measured, 0.2)
    assert np.allclose(out, measured)
    assert s.diagnostics.watchdog is True
    assert s.diagnostics.lease_expired is True
    assert "watchdog" in s.diagnostics.reasons
    assert "lease" in s.diagnostics.reasons
