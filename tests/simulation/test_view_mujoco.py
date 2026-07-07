"""Tests for scripts/view_mujoco.py interactive viewer helpers."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_VIEW_SCRIPT = _REPO_ROOT / "scripts" / "view_mujoco.py"


def test_dry_run_exits_zero() -> None:
    """--dry-run must load MJCF and build a trajectory without a display."""
    result = subprocess.run(
        [sys.executable, str(_VIEW_SCRIPT), "--dry-run"],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "dry-run ok" in result.stdout
    assert "ppp_warehouse" in result.stdout or "frames=" in result.stdout


def test_view_script_is_executable() -> None:
    """Wrapper shell script must exist next to view_mujoco.py."""
    view_sh = _REPO_ROOT / "scripts" / "view.sh"
    assert view_sh.is_file()
