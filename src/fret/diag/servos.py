"""``fret-diag servos`` — model / present-position sanity on the bus."""

from __future__ import annotations

import os

from fret.diag.core import CheckResult, CheckStatus


def run_servos_checks(port: str | None = None) -> list[CheckResult]:
    port = port or os.environ.get("FRET_SERVO_PORT", "mock:so101")
    try:
        from fret.hardware.cli import _open_bus
        from fret.hardware.sts.registers import PRESENT_POSITION

        client = _open_bus(port, 1_000_000)
        try:
            ids = client.scan(range(1, 7))
            positions: dict[int, int] = {}
            for sid in ids:
                raw = client.read(
                    sid, PRESENT_POSITION[0], PRESENT_POSITION[1]
                )
                positions[sid] = int.from_bytes(raw, "little")
        finally:
            client.close()
    except Exception as exc:  # noqa: BLE001
        return [
            CheckResult(
                "servos",
                (
                    CheckStatus.SKIP
                    if not port.startswith("mock:")
                    else CheckStatus.FAIL
                ),
                str(exc),
            )
        ]
    if len(ids) < 6 and port.startswith("mock:"):
        return [
            CheckResult(
                "servos",
                CheckStatus.FAIL,
                f"expected 6 mock servos, got {ids}",
            )
        ]
    if not ids:
        return [
            CheckResult(
                "servos",
                CheckStatus.SKIP,
                "no servos found",
            )
        ]
    return [
        CheckResult(
            "servos",
            CheckStatus.PASS,
            f"read positions for {list(positions)}",
            {"positions": positions},
        )
    ]
