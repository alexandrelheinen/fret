"""Tests for scripts/view_mujoco.py interactive viewer helpers."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_VIEW_SCRIPT = _REPO_ROOT / "scripts" / "view_mujoco.py"
_VIEW_CLI = [
    "--model",
    "dubins",
    "--scenario",
    "dubins_race",
    "--duration",
    "30",
    "--fps",
    "60",
    "--camera",
    "overview",
]


def test_dry_run_exits_zero() -> None:
    """--dry-run must load MJCF and build a trajectory without a display."""
    result = subprocess.run(
        [sys.executable, str(_VIEW_SCRIPT), *(_VIEW_CLI + ["--dry-run"])],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "dry-run ok" in result.stdout
    assert "dubins_race" in result.stdout or "frames=" in result.stdout


def test_view_cli_requires_explicit_args() -> None:
    """Invoking view_mujoco without args must fail with help."""
    result = subprocess.run(
        [sys.executable, str(_VIEW_SCRIPT)],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "missing arguments" in result.stderr


def test_view_script_requires_args() -> None:
    """Wrapper shell script must reject zero-arg invocation."""
    view_sh = _REPO_ROOT / "scripts" / "view.sh"
    result = subprocess.run(
        [str(view_sh)],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "missing arguments" in result.stderr
    assert "usage:" in result.stdout.lower()


def test_video_script_requires_args() -> None:
    """Wrapper shell script must reject zero-arg invocation."""
    video_sh = _REPO_ROOT / "scripts" / "video.sh"
    result = subprocess.run(
        [str(video_sh)],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "missing arguments" in result.stderr
    assert "usage:" in result.stdout.lower()


def test_view_script_is_executable() -> None:
    """Wrapper shell script must exist next to view_mujoco.py."""
    view_sh = _REPO_ROOT / "scripts" / "view.sh"
    assert view_sh.is_file()
