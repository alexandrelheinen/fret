"""Tests for scripts/render_mujoco.py (pure-Python helpers).

MuJoCo rendering is optional; these tests validate path resolution and
CLI parsing without a full showcase render.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

# render_mujoco.py lives under scripts/, not src/
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "scripts"
sys.path.insert(0, str(_SCRIPTS))

import render_mujoco as rm  # noqa: E402


def test_resolve_mjcf_override() -> None:
    explicit = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    resolved = rm.resolve_mjcf_path("dubins", "other", explicit)
    assert resolved == explicit


def test_resolve_mjcf_dubins_race() -> None:
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    assert path.name == "dubins_race.xml"
    assert path.is_file()


def test_resolve_mjcf_unsupported_raises() -> None:
    with pytest.raises(ValueError, match="Unsupported"):
        rm.resolve_mjcf_path("unknown", "missing", None)


def test_resolve_mjcf_omx_reach() -> None:
    path = rm.resolve_mjcf_path("open_manipulator_x", "omx_reach", None)
    assert path.name == "omx_tabletop.xml"
    assert path.is_file()


def test_resolve_mjcf_omy_pick_place() -> None:
    path = rm.resolve_mjcf_path("omy", "omy_pick_place", None)
    assert path.name.endswith("omy_pick_place.xml")
    assert path.is_file()


def test_resolve_mjcf_omy_clutter() -> None:
    path = rm.resolve_mjcf_path("omy", "omy_clutter", None)
    assert path.name.endswith("omy_clutter.xml")
    assert path.is_file()


def test_list_showcase_cameras_omy_pick_place() -> None:
    path = rm.resolve_mjcf_path("omy", "omy_pick_place", None)
    cameras = rm.list_showcase_cameras(path, scenario="omy_pick_place")
    assert cameras == ["overview"]


def test_robot_class_omy_scenarios() -> None:
    assert rm.robot_class_for_scenario("omy_pick_place") == "static"
    assert rm.robot_class_for_scenario("omy_clutter") == "static"


def test_list_showcase_cameras_reads_mjcf() -> None:
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    cameras = rm.list_showcase_cameras(path, scenario="dubins_race")
    assert cameras == ["overview", "follow"]


def test_list_showcase_cameras_omx_reach() -> None:
    path = rm.resolve_mjcf_path("open_manipulator_x", "omx_reach", None)
    cameras = rm.list_showcase_cameras(path, scenario="omx_reach")
    # Static arms: isometric overview only (no follow on release).
    assert cameras == ["overview"]


def test_robot_class_release_camera_policy() -> None:
    assert rm.robot_class_for_scenario("dubins_race") == "mobile"
    assert rm.robot_class_for_scenario("omx_pick_place") == "static"
    assert rm.release_cameras_for_scenario("dubins_race") == (
        "overview",
        "follow",
    )
    assert rm.release_cameras_for_scenario("omx_wall_maze_rrt") == (
        "overview",
    )
    assert rm.release_cameras_for_scenario("omx_wall_maze_sst") == (
        "overview",
    )


def test_resolve_mjcf_omy_clutter_planner_variants() -> None:
    for scenario in ("omy_clutter_rrt", "omy_clutter_sst"):
        path = rm.resolve_mjcf_path("omy", scenario, None)
        assert path.is_file()
        cameras = rm.list_showcase_cameras(path, scenario=scenario)
        assert cameras == ["overview"]


def test_resolve_mjcf_omx_wall_maze_planner_variants() -> None:
    for scenario in ("omx_wall_maze_rrt", "omx_wall_maze_sst"):
        path = rm.resolve_mjcf_path("open_manipulator_x", scenario, None)
        assert path.is_file()
        cameras = rm.list_showcase_cameras(path, scenario=scenario)
        assert cameras == ["overview"]


def test_dubins_race_mjcf_loads() -> None:
    """MJCF must load including race scene assets."""
    mujoco = pytest.importorskip("mujoco")
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq > 0


def test_showcase_output_name() -> None:
    assert rm.showcase_output_name("dubins_race", "overview") == (
        "dubins_race_overview.mp4"
    )


def test_showcase_playback_timing_identity() -> None:
    timing = rm.showcase_playback_timing(
        wall_sim_time_s=12.0,
        render_duration_s=12.0,
    )
    assert timing.sim_time_s == pytest.approx(12.0)
    assert timing.real_time_factor == pytest.approx(1.0)
    assert timing.wall_sim_time_s is None


def test_showcase_playback_timing_dubins_physics_cap() -> None:
    timing = rm.showcase_playback_timing(
        wall_sim_time_s=103.4,
        render_duration_s=35.0,
    )
    assert timing.sim_time_s == pytest.approx(35.0)
    assert timing.real_time_factor == pytest.approx(1.0)
    assert timing.wall_sim_time_s == pytest.approx(103.4)


def test_resolve_scenario_duration_reads_yaml() -> None:
    assert rm.resolve_scenario_duration("dubins_race") == pytest.approx(120.0)


_RELEASE_DUBINS_CLI: list[str] = [
    "--model",
    "dubins",
    "--scenario",
    "dubins_race",
    "--all-cameras",
    "--collision-backend",
    "mujoco",
    "--planner-algorithm",
    "sst",
    "--output-dir",
    "showcase_renders",
    "--timing-json",
    "showcase_renders/dubins_race_timing.json",
    "--full-duration",
    "--physics-mode",
    "--fps",
    "30",
    "--width",
    "1280",
    "--height",
    "720",
]


def test_resolve_physics_showcase_duration_caps_to_nominal() -> None:
    """V115-03: physics clips subsample long sim runs to scenario duration."""
    explicit = rm.resolve_physics_showcase_duration_s(
        "dubins_race",
        sim_time_s=180.0,
        duration_s=20.0,
    )
    assert explicit == pytest.approx(20.0)

    dubins_capped = rm.resolve_physics_showcase_duration_s(
        "dubins_race",
        sim_time_s=180.0,
        duration_s=None,
    )
    assert dubins_capped == pytest.approx(120.0)


def test_release_workflow_cli_args_parse() -> None:
    """Release workflow flags must parse without missing Namespace attributes."""
    parser = rm.build_parser()
    dubins_args = parser.parse_args(_RELEASE_DUBINS_CLI)

    assert dubins_args.model == "dubins"
    assert dubins_args.scenario == "dubins_race"
    assert dubins_args.all_cameras is True
    assert dubins_args.timing_json == Path(
        "showcase_renders/dubins_race_timing.json"
    )
    assert dubins_args.no_realtime_postprocess is False
    assert dubins_args.full_duration is True
    assert dubins_args.physics_mode is True


def test_render_cli_requires_explicit_args() -> None:
    """Invoking render_mujoco without args must fail with help."""
    result = subprocess.run(
        [sys.executable, str(_REPO_ROOT / "scripts" / "render_mujoco.py")],
        cwd=_REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "missing arguments" in result.stderr
    assert (
        "usage:" in result.stdout.lower() or "usage:" in result.stderr.lower()
    )


def test_main_accepts_release_dubins_cli(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """main() must accept the exact release.yml Dubins invocation."""
    timing = rm.ShowcaseTiming(sim_time_s=34.0, render_duration_s=34.0)

    def _fake_render(
        *_args: object, **_kwargs: object
    ) -> list[rm.RenderResult]:
        return [
            rm.RenderResult(
                camera="overview",
                path=Path("/tmp/dubins_race_overview.mp4"),
                frame_mean=42.0,
                timing=timing,
            )
        ]

    monkeypatch.setattr(rm, "render_showcase_videos", _fake_render)
    monkeypatch.setattr(rm, "postprocess_showcase_results", lambda r, **k: r)
    monkeypatch.setattr(
        rm, "write_showcase_timing_json", lambda *_a, **_k: None
    )

    assert rm.main(_RELEASE_DUBINS_CLI) == 0


def test_render_showcase_videos_rejects_unsupported_scenario() -> None:
    mjcf = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    with pytest.raises(ValueError, match="Unsupported showcase scenario"):
        rm.render_showcase_videos(
            mjcf,
            Path("/tmp"),
            scenario="unknown_scene",
        )
