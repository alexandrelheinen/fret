"""Package-share resource resolution (FR-SYS-10).

One function, :func:`resolve`, finds a file under
``<package share>/<kind>/<rel>`` through ``ament_index_python`` when
available (an installed workspace, from any working directory), falling
back to the package-relative source tree for pure-Python / editable-install
runs where the ROS overlay has not been sourced.

``sitl_config.resolve_package_file`` / ``package_share_root`` /
``package_source_root`` are kept as thin wrappers over this module so
existing call sites are unchanged.
"""

from __future__ import annotations

import pathlib


def package_source_root() -> pathlib.Path:
    """Return the on-disk ``fret`` Python package root (dev / editable installs)."""
    return pathlib.Path(__file__).resolve().parent


def package_share_root() -> pathlib.Path:
    """Return the installed ROS share directory for ``fret`` when available.

    Falls back to :func:`package_source_root` when ``ament_index_python``
    is unavailable or the package is not registered (pure-Python runs, or
    tests that have not sourced the ROS overlay).
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return pathlib.Path(get_package_share_directory("fret"))
    except Exception:
        return package_source_root()


def resolve(kind: str, rel: str) -> pathlib.Path:
    """Resolve ``<kind>/<rel>`` under the package share, or the source tree.

    Always checked in this order: the installed package share directory
    (independent of the current working directory), then the source tree
    (editable installs / pure-Python test runs). Never resolves relative
    to ``Path.cwd()`` (FR-SYS-10).

    Args:
        kind: Top-level resource category directory, e.g. ``"config"`` or
            ``"mjcf"``.
        rel: Path relative to ``kind``, e.g.
            ``"scenarios/omx_reach.yml"``.

    Returns:
        The first existing match.

    Raises:
        FileNotFoundError: If neither location has the file.
    """
    parts = (kind, *pathlib.PurePosixPath(rel).parts)
    seen: set[pathlib.Path] = set()
    for root in (package_share_root(), package_source_root()):
        if root in seen:
            continue
        seen.add(root)
        candidate = root.joinpath(*parts)
        if candidate.is_file():
            return candidate
    joined = "/".join(parts)
    raise FileNotFoundError(f"Package resource not found: {joined}")


def third_party_root(name: str) -> pathlib.Path:
    """Return the root of a declared third-party asset tree (e.g. Menagerie).

    PR1 note (tracked follow-up; see ``docs/decisions.md``): third-party
    submodules under ``third_party/`` are not yet installed by CMake, so
    this resolves only through the source tree today, unchanged from
    pre-PR1 behavior. A workspace installed without the source tree
    alongside it (no ``--symlink-install``, no sibling checkout) cannot
    yet resolve Menagerie assets this way; ``config``/``mjcf`` resolution
    via :func:`resolve` does not have this limitation.
    """
    return package_source_root().parents[1] / "third_party" / name
