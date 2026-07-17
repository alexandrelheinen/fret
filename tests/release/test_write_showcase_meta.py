"""Tests for scripts/release/write_showcase_meta.py."""

from __future__ import annotations

import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "release"))

from write_showcase_meta import write_showcase_meta  # noqa: E402


def _write_timing(path: Path, clips: list[dict[str, object]]) -> None:
    path.write_text(
        json.dumps({"clips": clips}, indent=2) + "\n",
        encoding="utf-8",
    )


def test_write_showcase_meta_dubins_artifacts(tmp_path: Path) -> None:
    """Publish step must accept Dubins artifacts in the renders folder."""
    renders = tmp_path / "showcase_renders"
    renders.mkdir()
    for name in (
        "dubins_race_overview.mp4",
        "dubins_race_follow.mp4",
    ):
        (renders / name).write_bytes(b"\x00" * 128)

    _write_timing(
        renders / "dubins_timing.json",
        [
            {
                "file": "dubins_race_overview.mp4",
                "sim_time_s": 32.0,
                "real_time_factor": 1.0,
            }
        ],
    )

    meta = write_showcase_meta(
        renders_dir=renders,
        tag="v9.9.9",
        repo="owner/fret",
        git_sha="abc123",
        workflow_run="https://example.com/run/1",
        output_path=tmp_path / "meta.json",
    )

    assert meta["git_ref"] == "v9.9.9"
    assert len(meta["showcases"]) == 1
    assert meta["primary_videos"]["dubins_race"] == "dubins_race_overview.mp4"
    assert (tmp_path / "meta.json").is_file()


def test_write_showcase_meta_accepts_single_scenario(tmp_path: Path) -> None:
    """Partial release uploads should still produce valid metadata."""
    renders = tmp_path / "showcase_renders"
    renders.mkdir()
    for name in ("dubins_race_overview.mp4", "dubins_race_follow.mp4"):
        (renders / name).write_bytes(b"\x00" * 64)

    _write_timing(
        renders / "dubins_timing.json",
        [
            {
                "file": "dubins_race_overview.mp4",
                "sim_time_s": 32.0,
                "real_time_factor": 1.0,
            }
        ],
    )

    meta = write_showcase_meta(
        renders_dir=renders,
        tag="v9.9.9",
        repo="owner/fret",
        git_sha="abc123",
        workflow_run="https://example.com/run/2",
        output_path=tmp_path / "meta.json",
    )

    assert meta["partial"] is False
    assert len(meta["showcases"]) == 1
    assert meta["showcases"][0]["scenario"] == "dubins_race"
