"""FR-SYS-16: vendored third-party files sit next to their upstream LICENSE,
and their ``IMPORTED_ASSETS.md`` records source URL, commit, license and
local edits.
"""

from __future__ import annotations

import pathlib

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
_SO101_DIR = _REPO_ROOT / "third_party" / "robotstudio_so101"

mujoco = pytest.importorskip("mujoco")


def test_so101_license_present_next_to_source() -> None:
    assert (_SO101_DIR / "LICENSE").is_file()
    assert (_SO101_DIR / "so101.xml").is_file()


def test_so101_imported_assets_records_provenance() -> None:
    text = (_SO101_DIR / "IMPORTED_ASSETS.md").read_text(encoding="utf-8")
    assert "github.com/google-deepmind/mujoco_menagerie" in text
    assert "Upstream commit:" in text
    assert "License:" in text
    assert "Local edits:" in text


def test_so101_mjcf_loads_in_mujoco() -> None:
    """The vendored scene must be a loadable, self-contained MJCF."""
    model = mujoco.MjModel.from_xml_path(str(_SO101_DIR / "so101.xml"))
    assert model.nu == 6
    assert model.nbody > 0
