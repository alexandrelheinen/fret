"""Tagged release symlink switching (FR-HW-56).

Layout::

    /opt/fret/releases/<tag>/   # complete install tree
    /opt/fret/current -> releases/<tag>
    /opt/fret/previous -> releases/<prev>   # optional bookmark

The ``current`` symlink is switched only after the caller confirms the
new release is complete (build + smoke). A failed build must leave
``current`` untouched.
"""

from __future__ import annotations

import os
from pathlib import Path


class ReleaseSwitchError(RuntimeError):
    """Raised when a release switch or rollback cannot proceed."""


def _resolve_link(path: Path) -> Path | None:
    if not path.is_symlink() and not path.exists():
        return None
    return path.resolve()


def activate_release(
    releases_dir: Path,
    current_link: Path,
    tag: str,
    *,
    previous_link: Path | None = None,
    require_marker: str = "install",
) -> Path:
    """Point ``current_link`` at ``releases_dir / tag`` after validation.

    Args:
        releases_dir: Directory containing per-tag release trees.
        current_link: Symlink path (``/opt/fret/current``).
        tag: Release tag directory name (for example ``v2.3.0-rc.1``).
        previous_link: Optional symlink updated to the prior current.
        require_marker: Relative path that must exist inside the release
            before the switch (default ``install``).

    Returns:
        Absolute path of the activated release directory.

    Raises:
        ReleaseSwitchError: If the release tree is incomplete.
    """
    release = (releases_dir / tag).resolve()
    marker = release / require_marker
    if not release.is_dir():
        raise ReleaseSwitchError(f"release directory missing: {release}")
    if not marker.exists():
        raise ReleaseSwitchError(
            f"release incomplete (missing {require_marker}): {release}"
        )

    releases_dir.mkdir(parents=True, exist_ok=True)
    current_link.parent.mkdir(parents=True, exist_ok=True)

    old = _resolve_link(current_link)
    if previous_link is not None and old is not None and old != release:
        previous_link.parent.mkdir(parents=True, exist_ok=True)
        tmp_prev = previous_link.with_name(previous_link.name + ".tmp")
        if tmp_prev.exists() or tmp_prev.is_symlink():
            tmp_prev.unlink()
        tmp_prev.symlink_to(old)
        os.replace(tmp_prev, previous_link)

    tmp = current_link.with_name(current_link.name + ".tmp")
    if tmp.exists() or tmp.is_symlink():
        tmp.unlink()
    tmp.symlink_to(release)
    os.replace(tmp, current_link)
    return release


def previous_release(previous_link: Path) -> Path | None:
    """Return the resolved previous release path, or ``None``."""
    return _resolve_link(previous_link)


def rollback_release(
    current_link: Path,
    previous_link: Path,
) -> Path:
    """Switch ``current_link`` back to the target of ``previous_link``.

    Raises:
        ReleaseSwitchError: If there is no previous release to restore.
    """
    prev = previous_release(previous_link)
    if prev is None:
        raise ReleaseSwitchError("no previous release to roll back to")
    if not prev.is_dir():
        raise ReleaseSwitchError(f"previous release missing: {prev}")

    # Swap: current becomes previous bookmark after rollback.
    old_current = _resolve_link(current_link)
    tmp = current_link.with_name(current_link.name + ".tmp")
    if tmp.exists() or tmp.is_symlink():
        tmp.unlink()
    tmp.symlink_to(prev)
    os.replace(tmp, current_link)
    if old_current is not None and old_current != prev:
        tmp_prev = previous_link.with_name(previous_link.name + ".tmp")
        if tmp_prev.exists() or tmp_prev.is_symlink():
            tmp_prev.unlink()
        tmp_prev.symlink_to(old_current)
        os.replace(tmp_prev, previous_link)
    return prev
