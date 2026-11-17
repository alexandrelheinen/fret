"""FR-HW-59: fret-diag pi with faked roots."""

from __future__ import annotations

from pathlib import Path

from fret.diag.core import CheckStatus
from fret.diag.pi import (
    check_architecture,
    check_free_disk,
    check_os_release,
    check_timer_loop,
    check_udev_symlinks,
    run_pi_checks,
)


def test_architecture_reports_machine() -> None:
    result = check_architecture()
    assert result.name == "architecture"
    assert result.status in {CheckStatus.PASS, CheckStatus.FAIL}


def test_os_release_pass_and_fail(tmp_path: Path) -> None:
    etc = tmp_path / "etc"
    etc.mkdir()
    (etc / "os-release").write_text(
        'PRETTY_NAME="Ubuntu 24.04.1 LTS"\nVERSION_ID="24.04"\n',
        encoding="utf-8",
    )
    assert check_os_release(tmp_path).status is CheckStatus.PASS
    (etc / "os-release").write_text(
        'PRETTY_NAME="Ubuntu 22.04.4 LTS"\nVERSION_ID="22.04"\n',
        encoding="utf-8",
    )
    assert check_os_release(tmp_path).status is CheckStatus.FAIL


def test_udev_symlinks_skip_when_absent(tmp_path: Path) -> None:
    dev = tmp_path / "dev"
    dev.mkdir()
    result = check_udev_symlinks(dev)
    assert result.status is CheckStatus.SKIP


def test_udev_symlinks_pass_when_present(tmp_path: Path) -> None:
    dev = tmp_path / "dev"
    dev.mkdir()
    (dev / "fret-servo-follower").touch()
    (dev / "fret-cam0").touch()
    assert check_udev_symlinks(dev).status is CheckStatus.PASS


def test_timer_loop_short() -> None:
    result = check_timer_loop(duration_s=0.2, rate_hz=50.0, p99_error_s=0.05)
    assert result.status is CheckStatus.PASS


def test_free_disk_runs() -> None:
    assert check_free_disk().status in {
        CheckStatus.PASS,
        CheckStatus.FAIL,
        CheckStatus.SKIP,
    }


def test_run_pi_checks_returns_named_results(tmp_path: Path) -> None:
    (tmp_path / "etc").mkdir()
    (tmp_path / "etc" / "os-release").write_text(
        'VERSION_ID="24.04"\nPRETTY_NAME="Ubuntu 24.04"\n',
        encoding="utf-8",
    )
    (tmp_path / "proc").mkdir()
    (tmp_path / "proc" / "meminfo").write_text(
        "MemTotal:       8192000 kB\n", encoding="utf-8"
    )
    (tmp_path / "proc" / "mounts").write_text(
        "/dev/mmcblk0p2 / ext4 rw,noatime 0 0\n", encoding="utf-8"
    )
    (tmp_path / "dev").mkdir()
    results = run_pi_checks(root=tmp_path, timer_duration_s=0.1)
    names = {r.name for r in results}
    assert "architecture" in names
    assert "arco" in names
    assert "fret" in names
