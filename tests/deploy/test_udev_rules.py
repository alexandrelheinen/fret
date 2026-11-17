"""FR-HW-58: udev rules template names and ModemManager ignore."""

from __future__ import annotations

from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_RULES = _REPO / "deploy" / "udev" / "99-fret.rules.in"


def test_udev_symlink_names_and_groups() -> None:
    text = _RULES.read_text(encoding="utf-8")
    assert 'SYMLINK+="fret-servo-follower"' in text
    assert 'SYMLINK+="fret-cam0"' in text
    assert "fret-servo-leader" in text  # reserved, commented
    assert 'GROUP="dialout"' in text
    assert 'GROUP="video"' in text
    assert 'MODE="0660"' in text
    assert 'ENV{ID_MM_DEVICE_IGNORE}="1"' in text


def test_udev_capture_script_exists() -> None:
    script = _REPO / "scripts" / "deploy" / "udev_capture.sh"
    assert script.is_file()
    body = script.read_text(encoding="utf-8")
    assert "--follower" in body
    assert "--camera" in body
    assert "--leader" in body
