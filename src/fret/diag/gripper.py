"""``fret-diag gripper`` — guarded gripper check (FR-HW-66)."""

from __future__ import annotations

from fret.diag.core import CheckResult, CheckStatus
from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock


def run_gripper_checks(
    *,
    allow_motion: bool,
    confirmed: bool,
    speed_scale: float = 0.3,
) -> list[CheckResult]:
    if not allow_motion:
        return [
            CheckResult(
                "gripper",
                CheckStatus.FAIL,
                "refused without --allow-motion",
            )
        ]
    if not confirmed:
        return [
            CheckResult(
                "gripper",
                CheckStatus.SKIP,
                "needs confirmation",
            )
        ]
    if speed_scale > 0.3:
        return [
            CheckResult(
                "gripper",
                CheckStatus.FAIL,
                f"speed_scale {speed_scale} > 0.3",
            )
        ]
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    if core.state is not BridgeState.CONNECTED_DISABLED:
        return [
            CheckResult(
                "gripper",
                CheckStatus.FAIL,
                f"bridge must be CONNECTED_DISABLED, got {core.state}",
            )
        ]
    return [
        CheckResult(
            "gripper",
            CheckStatus.PASS,
            "mock gripper guard path ok (no open/close on real hardware here)",
        )
    ]
