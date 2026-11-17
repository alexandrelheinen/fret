"""FR-HW-71: every mock fault has an observed bridge reaction."""

from __future__ import annotations

import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.sts.errors import (
    StsChecksumError,
    StsError,
    StsPortError,
    StsTimeoutError,
)
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind, make_so101_mock


def test_fault_corrupt_checksum_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.CORRUPT_CHECKSUM, remaining=1))
    with pytest.raises(StsChecksumError):
        core.cycle(0.0)
    assert core.diagnostics.integrity_rejects >= 1


def test_fault_drop_reply_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.DROP_REPLY, remaining=6))
    with pytest.raises(StsTimeoutError):
        core.cycle(0.0)
    assert core.diagnostics.timeouts >= 1


def test_fault_wrong_id_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.WRONG_ID, remaining=1))
    with pytest.raises(StsError):
        core.cycle(0.0)


def test_fault_missing_servo_fr_hw_71() -> None:
    bus = make_so101_mock()
    bus.inject_fault(FaultConfig(FaultKind.MISSING_SERVO, servo_id=2))
    core, _ = make_bridge_core_for_mock(bus=bus)
    with pytest.raises(StsError):
        core.connect()


def test_fault_low_voltage_surfaced_in_diagnostics_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.LOW_VOLTAGE, servo_id=1))
    core.cycle(0.0)
    assert bus.servo(1).voltage == 50
    assert core.diagnostics.servo_error_counts.get(1, 0) >= 1


def test_fault_overload_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.OVERLOAD, servo_id=1))
    core.cycle(0.0)
    assert bus.servo(1).status & 0x20
    assert core.diagnostics.servo_error_counts.get(1, 0) >= 1


def test_fault_overheat_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.OVERHEAT, servo_id=1))
    core.cycle(0.0)
    assert bus.servo(1).temperature == 85
    assert core.diagnostics.servo_error_counts.get(1, 0) >= 1


def test_fault_cable_pull_disconnects_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(FaultKind.CABLE_PULL))
    for i in range(5):
        with pytest.raises(StsTimeoutError):
            core.cycle(float(i) * 0.02)
    # PR4: sustained bus silence → FAULT (power_loss path).
    assert core.state is BridgeState.FAULT
    assert core.diagnostics.timeouts >= 1


def test_fault_latency_still_completes_fr_hw_71() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(
        FaultConfig(FaultKind.LATENCY, latency_s=0.01, remaining=1)
    )
    core.cycle(0.0)
    assert core.state is BridgeState.CONNECTED_DISABLED
