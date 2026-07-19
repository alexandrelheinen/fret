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
    }
    assert manifest.fps == 30
    assert manifest.width == 1280
    assert manifest.height == 720


def test_omx_release_variants_differ_only_by_planner() -> None:
    manifest = load_showcase_manifest()
    rrt = manifest.by_id("omx_wall_maze_rrt")
    sst = manifest.by_id("omx_wall_maze_sst")
    assert rrt.robot_class == sst.robot_class == "static"
    assert rrt.cameras == sst.cameras == ("overview",)
    assert rrt.planner_algorithm == "rrt_star"
    assert sst.planner_algorithm == "sst"


def test_mobile_requires_follow_static_overview_only() -> None:
    manifest = load_showcase_manifest()
    for item in manifest.scenarios:
        if item.robot_class == "mobile":
            assert item.cameras == ("overview", "follow")
            assert item.requires_follow is True
        else:
            assert item.cameras == ("overview",)
            assert item.requires_follow is False


def test_release_cameras_for_robot_class() -> None:
    assert release_cameras_for_robot_class("mobile") == ("overview", "follow")
    assert release_cameras_for_robot_class("static") == ("overview",)
