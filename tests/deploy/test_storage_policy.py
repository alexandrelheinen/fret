"""FR-HW-72: microSD storage policy artifacts."""

from __future__ import annotations

from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]


def test_journald_cap() -> None:
    text = (_REPO / "deploy" / "journald" / "fret.conf").read_text(
        encoding="utf-8"
    )
    assert "Storage=volatile" in text or "SystemMaxUse=" in text
    assert "100M" in text or "volatile" in text


def test_tmpfs_logs_and_zram() -> None:
    fstab = (_REPO / "deploy" / "fstab.d" / "fret-tmpfs.in").read_text(
        encoding="utf-8"
    )
    assert "/var/log/fret" in fstab
    assert "size=256M" in fstab
    zram = (_REPO / "deploy" / "zram" / "zram-generator.conf").read_text(
        encoding="utf-8"
    )
    assert "[zram0]" in zram
    install = (_REPO / "scripts" / "deploy" / "rpi5_install.sh").read_text(
        encoding="utf-8"
    )
    assert "zram" in install
    assert "swapfile" in install.lower()
