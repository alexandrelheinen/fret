"""``fret-diag usb`` — USB device presence (FR-HW-65)."""

from __future__ import annotations

from pathlib import Path

from fret.diag.core import CheckResult, CheckStatus


def run_usb_checks(dev_root: Path | None = None) -> list[CheckResult]:
    root = dev_root or Path("/dev")
    results: list[CheckResult] = []
    for name, label in (
        ("fret-servo-follower", "follower adapter"),
        ("fret-cam0", "camera"),
    ):
        path = root / name
        if path.exists():
            results.append(
                CheckResult(
                    f"usb_{name}",
                    CheckStatus.PASS,
                    f"{label} present at {path}",
                )
            )
        else:
            results.append(
                CheckResult(
                    f"usb_{name}",
                    CheckStatus.SKIP,
                    f"{label} absent at {path}",
                )
            )
    return results
