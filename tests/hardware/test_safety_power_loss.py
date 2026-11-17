"""FR-HW-40: bus power-loss → FAULT within 5 cycles; no auto re-enable."""

from __future__ import annotations

from pathlib import Path

import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.sts.errors import StsError, StsTimeoutError
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind


def test_power_cut_enters_fault_within_five_cycles_fr_hw_40() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.POWER_CUT))
    assert bus.power_cut is True
    for i in range(5):
        with pytest.raises(StsTimeoutError):
            core.cycle(float(i) * 0.02)
    assert core.state is BridgeState.FAULT
    assert core.diagnostics.fault_reason == "power_loss"


def test_power_restore_does_not_auto_enable_fr_hw_40() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.POWER_CUT))
    for i in range(5):
        with pytest.raises(StsTimeoutError):
            core.cycle(float(i) * 0.02)
    assert core.state is BridgeState.FAULT
    bus.clear_faults()
    # Power is back; still FAULT until clear + enable.
    core.cycle(1.0)
    assert core.state is BridgeState.FAULT
    with pytest.raises(StsError, match="clear"):
        core.enable_torque(1.1)
    core.clear()
    assert core.state is BridgeState.CONNECTED_DISABLED
    core.enable_torque(1.2)
    assert core.state is BridgeState.ENABLED


def test_clear_while_power_absent_stays_fault_fr_hw_40() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.POWER_CUT))
    for i in range(5):
        with pytest.raises(StsTimeoutError):
            core.cycle(float(i) * 0.02)
    core.clear()
    assert core.state is BridgeState.FAULT


def test_safety_docs_describe_both_stop_configs_fr_hw_40() -> None:
    path = (
        Path(__file__).resolve().parents[2] / "docs" / "hardware" / "safety.md"
    )
    text = path.read_text(encoding="utf-8")
    assert "Case (a)" in text or "case (a)" in text.lower()
    assert "Case (b)" in text or "case (b)" in text.lower()
    assert "e-stop" in text.lower() or "estop" in text.lower()
    assert "arm goes limp" in text
    assert "emergency stop" in text.lower()
    # Must not present ~/stop as emergency.
    assert (
        "not an emergency" in text.lower()
        or "never an emergency" in text.lower()
    )
