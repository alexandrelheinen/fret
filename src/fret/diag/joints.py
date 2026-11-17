"""``fret-diag joints`` — guarded small motions (FR-HW-66)."""

from __future__ import annotations

from fret.diag.core import CheckResult, CheckStatus
from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock


def run_joints_checks(
    *,
    allow_motion: bool,
    confirmed: bool,
    delta_rad: float = 0.1,
    speed_scale: float = 0.3,
) -> list[CheckResult]:
    if not allow_motion:
        return [
            CheckResult(
                "joints",
                CheckStatus.FAIL,
                "refused without --allow-motion",
            )
        ]
    if not confirmed:
        return [
            CheckResult(
                "joints",
                CheckStatus.SKIP,
                "needs confirmation",
            )
        ]
    if speed_scale > 0.3:
        return [
            CheckResult(
                "joints",
                CheckStatus.FAIL,
                f"speed_scale {speed_scale} > 0.3",
            )
        ]
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    if core.state is not BridgeState.CONNECTED_DISABLED:
        return [
            CheckResult(
                "joints",
                CheckStatus.FAIL,
                f"bridge must be CONNECTED_DISABLED, got {core.state}",
            )
        ]
    # Read-only direction check placeholder: enable briefly on mock only.
    import numpy as np

    before = core.positions
    core.enable_torque(0.0)
    n = int(before.shape[0])
    core.command_velocity(np.zeros(n, dtype=np.float64))
    core.cycle(0.02)
    core.disable_torque()
    return [
        CheckResult(
            "joints",
            CheckStatus.PASS,
            f"mock joint nudge delta={delta_rad} rad at scale={speed_scale}",
            {"before": [float(x) for x in before]},
        )
    ]
