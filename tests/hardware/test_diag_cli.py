"""FR-HW-65: fret-diag CLI exit codes, JSON, ordering."""

from __future__ import annotations

from pathlib import Path

from fret.diag.cli import main
from fret.diag.core import READ_ONLY_ORDER


def test_pi_writes_json_and_exits(tmp_path: Path) -> None:
    rc = main(
        [
            "pi",
            "--report-dir",
            str(tmp_path),
            "--timer-duration",
            "0.1",
        ]
    )
    assert rc in {0, 1, 2}
    reports = list(tmp_path.glob("*-pi.json"))
    assert len(reports) == 1
    body = reports[0].read_text(encoding="utf-8")
    assert '"command": "pi"' in body


def test_all_is_read_only_order() -> None:
    assert "joints" not in READ_ONLY_ORDER
    assert "gripper" not in READ_ONLY_ORDER
    assert "e2e" not in READ_ONLY_ORDER
    assert READ_ONLY_ORDER[0] == "pi"


def test_motion_refused_without_allow(tmp_path: Path) -> None:
    rc = main(["joints", "--report-dir", str(tmp_path)])
    assert rc == 1
