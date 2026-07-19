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


def _stub_clip(renders: Path, name: str, *, bytes_len: int = 128) -> None:
    (renders / name).write_bytes(b"\x00" * bytes_len)


def test_write_showcase_meta_all_scenarios(tmp_path: Path) -> None:
    """Publish step must accept the full multi-scenario render bundle."""
    renders = tmp_path / "showcase_renders"
    renders.mkdir()

    clips: list[tuple[str, str, str]] = [
        ("dubins_race", "overview", "dubins_race_timing.json"),
        ("dubins_race", "follow", "dubins_race_timing.json"),
        ("omx_wall_maze_rrt", "overview", "omx_wall_maze_rrt_timing.json"),
        ("omx_wall_maze_sst", "overview", "omx_wall_maze_sst_timing.json"),
        ("omy_pick_place", "overview", "omy_pick_place_timing.json"),
        ("omy_clutter_rrt", "overview", "omy_clutter_rrt_timing.json"),
        ("omy_clutter_sst", "overview", "omy_clutter_sst_timing.json"),
    ]
    timing_by_file: dict[str, list[dict[str, object]]] = {}
    for scenario, camera, timing_name in clips:
        mp4 = f"{scenario}_{camera}.mp4"
        _stub_clip(renders, mp4)
        timing_by_file.setdefault(timing_name, []).append(
            {
                "file": mp4,
                "sim_time_s": 12.0,
                "real_time_factor": 1.0,
            }
        )
    for timing_name, entries in timing_by_file.items():
        _write_timing(renders / timing_name, entries)

    meta = write_showcase_meta(
        renders_dir=renders,
        tag="v1.2.3",
        repo="owner/fret",
        git_sha="abc123",
        workflow_run="https://example.com/run/1",
        output_path=tmp_path / "meta.json",
    )

    assert meta["git_ref"] == "v1.2.3"
    assert meta["partial"] is False
    assert len(meta["showcases"]) == 6
    assert meta["release_cameras"] == {
        "mobile": ["overview", "follow"],
        "static": ["overview"],
    }
    assert meta["primary_videos"]["dubins_race"] == "dubins_race_overview.mp4"
    assert meta["primary_videos"]["omx_wall_maze_rrt"] == (
        "omx_wall_maze_rrt_overview.mp4"
    )
    assert meta["primary_videos"]["omx_wall_maze_sst"] == (
        "omx_wall_maze_sst_overview.mp4"
    )
    assert meta["primary_videos"]["omy_pick_place"] == (
        "omy_pick_place_overview.mp4"
    )
    assert meta["primary_videos"]["omy_clutter_rrt"] == (
        "omy_clutter_rrt_overview.mp4"
    )
    assert meta["primary_videos"]["omy_clutter_sst"] == (
        "omy_clutter_sst_overview.mp4"
    )
    assert (tmp_path / "meta.json").is_file()


def test_write_showcase_meta_accepts_partial_bundle(tmp_path: Path) -> None:
    """Partial uploads (e.g. one scenario) still produce valid metadata."""
    renders = tmp_path / "showcase_renders"
    renders.mkdir()
    for name in ("dubins_race_overview.mp4", "dubins_race_follow.mp4"):
        _stub_clip(renders, name, bytes_len=64)

    _write_timing(
        renders / "dubins_race_timing.json",
        [
            {
                "file": "dubins_race_overview.mp4",
                "sim_time_s": 32.0,
                "real_time_factor": 1.0,
            },
            {
                "file": "dubins_race_follow.mp4",
                "sim_time_s": 32.0,
                "real_time_factor": 1.0,
            },
        ],
    )

    meta = write_showcase_meta(
        renders_dir=renders,
        tag="v1.2.3-dev",
        repo="owner/fret",
        git_sha="abc123",
        workflow_run="https://example.com/run/2",
        output_path=tmp_path / "meta.json",
    )

    assert meta["partial"] is True
    assert len(meta["showcases"]) == 1
    assert meta["showcases"][0]["scenario"] == "dubins_race"
    assert meta["showcases"][0]["robot_class"] == "mobile"
