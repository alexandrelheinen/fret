"""``fret-diag calibration`` — machine-local calibration files present."""

from __future__ import annotations

from pathlib import Path

from fret.diag.core import CheckResult, CheckStatus


def run_calibration_checks(
    calib_dir: Path | None = None,
) -> list[CheckResult]:
    directory = calib_dir or Path("/var/lib/fret/calibration")
    if not directory.is_dir():
        return [
            CheckResult(
                "calibration",
                CheckStatus.SKIP,
                f"calibration dir missing: {directory}",
            )
        ]
    files = sorted(directory.glob("*.yml")) + sorted(directory.glob("*.yaml"))
    if not files:
        return [
            CheckResult(
                "calibration",
                CheckStatus.SKIP,
                f"no YAML calibrations in {directory}",
            )
        ]
    return [
        CheckResult(
            "calibration",
            CheckStatus.PASS,
            f"{len(files)} calibration file(s)",
            {"files": [p.name for p in files]},
        )
    ]
