"""``fret-diag bus`` — Feetech bus ping via mock or live port."""

from __future__ import annotations

import os

from fret.diag.core import CheckResult, CheckStatus


def run_bus_checks(port: str | None = None) -> list[CheckResult]:
    port = port or os.environ.get("FRET_SERVO_PORT", "mock:so101")
    try:
        from fret.hardware.cli import _open_bus

        client = _open_bus(port, 1_000_000)
        try:
            found = client.scan(range(1, 7))
        finally:
            client.close()
    except Exception as exc:  # noqa: BLE001 — surface as fail/skip
        if port.startswith("mock:"):
            return [CheckResult("bus", CheckStatus.FAIL, str(exc))]
        return [
            CheckResult(
                "bus",
                CheckStatus.SKIP,
                f"bus open failed ({exc}); hardware may be absent",
            )
        ]
    if not found:
        return [
            CheckResult(
                "bus",
                CheckStatus.FAIL,
                f"no servos answered on {port}",
            )
        ]
    return [
        CheckResult(
            "bus",
            CheckStatus.PASS,
            f"servos {found} on {port}",
            {"ids": found, "port": port},
        )
    ]
