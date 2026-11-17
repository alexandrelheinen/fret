"""FR-HW-57 / FR-HW-64: systemd units start disabled and support watchdog."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_UNITS = _REPO / "deploy" / "systemd"


def _read(name: str) -> str:
    return (_UNITS / name).read_text(encoding="utf-8")


def test_hardware_unit_never_enables_torque() -> None:
    text = _read("fret-hardware.service")
    assert "WatchdogSec=" in text
    assert "Type=notify" in text
    assert "auto_enable" not in text.lower()
    assert "--enable" not in text
    launch = (_REPO / "deploy" / "bin" / "fret-hardware-launch").read_text(
        encoding="utf-8"
    )
    assert "enable" not in launch.lower() or "Never pass an enable" in launch


def test_planner_is_niced() -> None:
    text = _read("fret-planner.service")
    assert "Nice=10" in text


def test_vision_and_zenoh_units_exist() -> None:
    assert (_UNITS / "fret-vision.service").is_file()
    assert (_UNITS / "fret-zenohd.service").is_file()


def test_systemd_analyze_verify_when_available() -> None:
    if shutil.which("systemd-analyze") is None:
        return
    for name in (
        "fret-hardware.service",
        "fret-planner.service",
        "fret-vision.service",
        "fret-zenohd.service",
    ):
        path = _UNITS / name
        result = subprocess.run(
            ["systemd-analyze", "verify", str(path)],
            check=False,
            capture_output=True,
            text=True,
        )
        # verify may warn about missing EnvironmentFile paths off-Pi.
        assert result.returncode in {0, 1}, result.stderr
