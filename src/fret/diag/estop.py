"""``fret-diag estop`` — power-loss → FAULT, no auto re-enable (FR-HW-40)."""

from __future__ import annotations

from fret.diag.core import CheckResult, CheckStatus
from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.sts.errors import StsTimeoutError
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind


def run_estop_checks() -> list[CheckResult]:
    """Exercise mock power-loss path without moving real hardware."""
    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    bus.inject_fault(FaultConfig(FaultKind.POWER_CUT))
    for i in range(5):
        try:
            core.cycle(float(i) * 0.02)
        except StsTimeoutError:
            pass
    if core.state != BridgeState.FAULT:
        return [
            CheckResult(
                "estop",
                CheckStatus.FAIL,
                f"expected FAULT after power loss, got {core.state}",
            )
        ]
    bus.clear_faults()
    try:
        core.cycle(1.0)
    except StsTimeoutError:
        pass
    # Compare by name so mypy does not narrow Enum identity across cycles.
    if core.state.name == BridgeState.ENABLED.name:
        return [
            CheckResult(
                "estop",
                CheckStatus.FAIL,
                "auto re-enabled after power return",
            )
        ]
    return [
        CheckResult(
            "estop",
            CheckStatus.PASS,
            "FAULT on power loss; stayed disabled after restore",
        )
    ]
