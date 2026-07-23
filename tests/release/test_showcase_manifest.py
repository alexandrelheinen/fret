"""Tests for scripts/release/showcase_manifest.py."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "release"))

from showcase_manifest import (  # noqa: E402
    load_showcase_manifest,
    release_cameras_for_robot_class,
)


def test_manifest_loads_release_focus_scenarios() -> None:
    manifest = load_showcase_manifest()
    ids = {item.id for item in manifest.scenarios}
    assert ids == {
        "dubins_race",
        "omx_wall_maze_rrt",
        "omx_wall_maze_sst",
        "omy_pick_place",
        "omy_clutter_rrt",
        "omy_clutter_sst",
    }
    assert manifest.fps == 30
    assert manifest.width == 1280
    assert manifest.height == 720


def test_dubins_release_shows_finish_with_hold_and_lower_res() -> None:
    """Dubins: full race → fixed 40 s video @ 960×540 / 20 fps (no follow)."""
    dubins = load_showcase_manifest().by_id("dubins_race")
    assert dubins.cameras == ("overview",)
    assert dubins.requires_follow is False
    assert dubins.fps == 20
    assert dubins.width == 960
    assert dubins.height == 540
    assert dubins.clip_duration_s == 40.0
    assert dubins.clip_scale is None
    assert dubins.effective_fps(30) == 20
    assert dubins.effective_width(1280) == 960
    assert dubins.effective_height(720) == 540
    assert dubins.physics_mode is True


def test_omy_release_variants_differ_only_by_planner() -> None:
    manifest = load_showcase_manifest()
    rrt = manifest.by_id("omy_clutter_rrt")
    sst = manifest.by_id("omy_clutter_sst")
    assert rrt.robot_class == sst.robot_class == "static"
    assert rrt.cameras == sst.cameras == ("overview",)
    assert rrt.planner_algorithm == "rrt_star"
    assert sst.planner_algorithm == "sst"


def test_omx_release_variants_differ_only_by_planner() -> None:
    manifest = load_showcase_manifest()
    rrt = manifest.by_id("omx_wall_maze_rrt")
    sst = manifest.by_id("omx_wall_maze_sst")
    assert rrt.robot_class == sst.robot_class == "static"
    assert rrt.cameras == sst.cameras == ("overview",)
    assert rrt.planner_algorithm == "rrt_star"
    assert sst.planner_algorithm == "sst"


def test_overview_required_follow_optional_for_mobile() -> None:
    manifest = load_showcase_manifest()
    for item in manifest.scenarios:
        assert "overview" in item.cameras
        if item.robot_class == "static":
            assert "follow" not in item.cameras
            assert item.requires_follow is False


def test_release_cameras_for_robot_class() -> None:
    assert release_cameras_for_robot_class("mobile") == ("overview",)
    assert release_cameras_for_robot_class("static") == ("overview",)
