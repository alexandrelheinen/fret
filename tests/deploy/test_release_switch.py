"""FR-HW-56: tagged release switch and rollback leave current safe."""

from __future__ import annotations

from pathlib import Path

import pytest

from fret.deploy.release_switch import (
    ReleaseSwitchError,
    activate_release,
    rollback_release,
)


def _make_release(root: Path, tag: str, *, complete: bool = True) -> Path:
    release = root / "releases" / tag
    release.mkdir(parents=True)
    if complete:
        (release / "install").mkdir()
        (release / "install" / "ok").write_text("1", encoding="utf-8")
    return release


def test_activate_switches_current_only_when_complete(tmp_path: Path) -> None:
    releases = tmp_path / "releases"
    current = tmp_path / "current"
    previous = tmp_path / "previous"
    _make_release(tmp_path, "v1.0.0")
    activate_release(releases, current, "v1.0.0", previous_link=previous)
    assert current.resolve() == (releases / "v1.0.0").resolve()

    _make_release(tmp_path, "v1.1.0")
    activate_release(releases, current, "v1.1.0", previous_link=previous)
    assert current.resolve() == (releases / "v1.1.0").resolve()
    assert previous.resolve() == (releases / "v1.0.0").resolve()


def test_incomplete_release_leaves_current_untouched(tmp_path: Path) -> None:
    releases = tmp_path / "releases"
    current = tmp_path / "current"
    _make_release(tmp_path, "v1.0.0")
    activate_release(releases, current, "v1.0.0")
    before = current.resolve()
    _make_release(tmp_path, "v1.2.0-bad", complete=False)
    with pytest.raises(ReleaseSwitchError, match="incomplete"):
        activate_release(releases, current, "v1.2.0-bad")
    assert current.resolve() == before


def test_rollback_restores_previous(tmp_path: Path) -> None:
    releases = tmp_path / "releases"
    current = tmp_path / "current"
    previous = tmp_path / "previous"
    _make_release(tmp_path, "v1.0.0")
    _make_release(tmp_path, "v1.1.0")
    activate_release(releases, current, "v1.0.0", previous_link=previous)
    activate_release(releases, current, "v1.1.0", previous_link=previous)
    rolled = rollback_release(current, previous)
    assert rolled == (releases / "v1.0.0").resolve()
    assert current.resolve() == (releases / "v1.0.0").resolve()
