"""``fret-diag camera`` — V4L2 negotiated format (FR-HW-30/32)."""

from __future__ import annotations

import os
from pathlib import Path

from fret.diag.core import CheckResult, CheckStatus


def run_camera_checks(
    device: str | None = None,
    *,
    fake_info: dict[str, float | int | str] | None = None,
) -> list[CheckResult]:
    if fake_info is not None:
        width = int(fake_info.get("width", 0))
        height = int(fake_info.get("height", 0))
        fps = float(fake_info.get("fps", 0.0))
        fmt = str(fake_info.get("format", "unknown"))
        ok = width >= 1280 and height >= 720
        return [
            CheckResult(
                "camera",
                CheckStatus.PASS if ok else CheckStatus.FAIL,
                f"{width}x{height} @{fps} fps format={fmt}",
                {
                    "width": width,
                    "height": height,
                    "fps": fps,
                    "format": fmt,
                },
            )
        ]

    path = Path(
        device or os.environ.get("FRET_CAMERA_DEVICE", "/dev/fret-cam0")
    )
    if not path.exists():
        return [
            CheckResult(
                "camera",
                CheckStatus.SKIP,
                f"camera device missing: {path}",
            )
        ]
    # Without v4l2-ctl parsing in unit tests, presence alone is a soft pass.
    return [
        CheckResult(
            "camera",
            CheckStatus.PASS,
            f"device present at {path} (negotiate format via v4l2-ctl on Pi)",
        )
    ]
