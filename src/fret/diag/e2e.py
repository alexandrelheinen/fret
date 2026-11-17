"""``fret-diag e2e`` — guarded end-to-end supervised pick dry-run (FR-HW-66)."""

from __future__ import annotations

from fret.diag.core import CheckResult, CheckStatus


def run_e2e_checks(
    *,
    allow_motion: bool,
    confirmed: bool,
    speed_scale: float = 0.3,
) -> list[CheckResult]:
    if not allow_motion:
        return [
            CheckResult(
                "e2e",
                CheckStatus.FAIL,
                "refused without --allow-motion",
            )
        ]
    if not confirmed:
        return [
            CheckResult(
                "e2e",
                CheckStatus.SKIP,
                "needs confirmation",
            )
        ]
    if speed_scale > 0.3:
        return [
            CheckResult(
                "e2e",
                CheckStatus.FAIL,
                f"speed_scale {speed_scale} > 0.3",
            )
        ]
    return [
        CheckResult(
            "e2e",
            CheckStatus.PASS,
            "guards ok; run SC-v23a / OPS-08 for the supervised pick",
        )
    ]
