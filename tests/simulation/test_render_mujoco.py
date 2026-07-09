"""Tests for scripts/render_mujoco.py (pure-Python helpers).

MuJoCo rendering is optional; these tests validate trajectory generation
and MJCF path resolution without a MuJoCo runtime.
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


def test_resolve_mjcf_ppp_warehouse() -> None:
    path = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    assert path.name == "ppp_warehouse.xml"
    assert path.is_file()


def test_resolve_mjcf_override() -> None:
    explicit = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    resolved = rm.resolve_mjcf_path("ppp", "other", explicit)
    assert resolved == explicit


def test_resolve_mjcf_dubins_race() -> None:
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    assert path.name == "dubins_race.xml"
    assert path.is_file()


def test_resolve_mjcf_unsupported_raises() -> None:
    with pytest.raises(ValueError, match="Unsupported"):
        rm.resolve_mjcf_path("unknown", "missing", None)


def test_interpolate_waypoints_shape() -> None:
    waypoints = [
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
        np.array([2.0, 0.5, 2.0]),
    ]
    traj = rm.interpolate_waypoints(waypoints, duration_s=3.0, fps=10)
    assert traj.shape == (30, 3)
    assert traj[0, 0] == pytest.approx(0.0)
    assert traj[-1, 0] == pytest.approx(2.0)


def test_interpolate_waypoints_respects_limits() -> None:
    waypoints = [
        np.array([0.0, 0.0, 0.0]),
        np.array([20.0, 10.0, 10.0]),
    ]
    traj = rm.interpolate_waypoints(waypoints, duration_s=1.0, fps=5)
    assert np.all(traj[:, 0] <= rm._PPP_LIMITS[0, 1])
    assert np.all(traj[:, 1] <= rm._PPP_LIMITS[1, 1])
    assert np.all(traj[:, 2] <= rm._PPP_LIMITS[2, 1])


def test_list_showcase_cameras_reads_mjcf() -> None:
    path = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    cameras = rm.list_showcase_cameras(path)
    assert cameras == ["overview", "follow"]


def test_ppp_warehouse_mjcf_loads_with_aws_meshes() -> None:
    """MJCF must load including vendored AWS warehouse mesh assets."""
    mujoco = pytest.importorskip("mujoco")
    path = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nmesh >= 3


def test_showcase_output_name() -> None:
    assert rm.showcase_output_name("ppp_warehouse", "aisle") == (
        "ppp_warehouse_aisle.mp4"
    )


def test_build_showcase_waypoints_mujoco_backend() -> None:
    """Release renders plan via MuJoCo collision checking and RRT*."""
    pytest.importorskip("mujoco")
    pytest.importorskip("arco")
    mjcf = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    waypoints = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        planner_algorithm="rrt_star",
        mjcf_path=mjcf,
    )
    assert len(waypoints) >= 4
    assert waypoints[0][0] == pytest.approx(2.0)
    assert waypoints[-1][0] == pytest.approx(10.5)


def test_showcase_cruise_avoids_flying_over_floor_clutter() -> None:
    """Transit must route around racks, not pass through obstacle footprints."""
    pytest.importorskip("mujoco")
    pytest.importorskip("arco")
    from fret.planning.ppp_obstacles import load_ppp_warehouse_preview_obstacles

    mjcf = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    waypoints = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        mjcf_path=mjcf,
    )
    boxes = load_ppp_warehouse_preview_obstacles()
    max_obs_z = max(box.z_max for box in boxes)
    cruise_z = float(waypoints[2][2])
    assert cruise_z < max_obs_z + 0.6
    for q in waypoints:
        if abs(float(q[2]) - cruise_z) > 0.02:
            continue
        for box in boxes[:3]:
            inside_xy = (
                box.x_min <= float(q[0]) <= box.x_max
                and box.y_min <= float(q[1]) <= box.y_max
            )
            assert not inside_xy, (
                f"cruise waypoint ({q[0]:.2f}, {q[1]:.2f}) crosses "
                f"obstacle footprint at z={cruise_z:.2f}"
            )


def test_simulate_tracked_trajectory_covers_horizontal_transit() -> None:
    """Tracked showcase clips must traverse X/Y, not stall in the pick segment."""
    pytest.importorskip("mujoco")
    pytest.importorskip("arco")
    mjcf = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    path = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        mjcf_path=mjcf,
    )
    dense = rm.build_pruned_dense_waypoints(path)
    traj, sim_time_s = rm.simulate_tracked_trajectory(
        dense,
        duration_s=None,
        fps=30,
        start=path[0],
    )
    assert traj.shape[0] >= 2
    assert sim_time_s > 0.0
    assert traj[:, 0].max() > 8.0
    assert traj[:, 1].max() > 2.0


def test_interpolated_dense_showcase_covers_horizontal_transit() -> None:
    """Release-style interpolation must span the planned warehouse transit."""
    pytest.importorskip("mujoco")
    pytest.importorskip("arco")
    mjcf = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    path = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        mjcf_path=mjcf,
    )
    traj, sim_time_s = rm.build_showcase_trajectory(
        path,
        scenario="ppp_warehouse",
        duration_s=None,
        fps=30,
        collision_backend="mujoco",
        use_tracking=False,
    )
    assert traj.shape[0] >= 2
    assert sim_time_s > 0.0
    assert traj[:, 0].max() > 8.0
    assert traj[:, 1].max() > 2.0
    assert traj[:, 2].max() - traj[:, 2].min() > 1.5


def test_pick_place_waypoints_include_vertical_segments() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("arco")
    mjcf = rm.resolve_mjcf_path("ppp", "ppp_warehouse", None)
    path = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        mjcf_path=mjcf,
    )
    z_vals = [float(q[2]) for q in path]
    assert min(z_vals) < 1.0
    assert max(z_vals) > 2.0


def test_interpolate_segmented_waypoints_preserves_z_motion() -> None:
    waypoints = [
        np.array([2.0, 1.0, 2.4]),
        np.array([2.0, 1.0, 0.6]),
        np.array([2.0, 1.0, 2.2]),
        np.array([10.5, 2.8, 2.2]),
        np.array([10.5, 2.8, 0.6]),
        np.array([10.5, 2.8, 2.65]),
    ]
    durations = rm.pick_place_segment_durations(waypoints)
    traj, sim_time_s = rm.interpolate_segmented_waypoints(
        waypoints,
        durations,
        fps=30,
    )
    assert sim_time_s > 10.0
    assert traj[:, 2].max() - traj[:, 2].min() > 1.5


def test_list_showcase_cameras_dubins_race() -> None:
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    cameras = rm.list_showcase_cameras(path, scenario="dubins_race")
    assert cameras == ["overview", "follow"]


def test_dubins_race_mjcf_loads_with_warehouse_assets() -> None:
    """MJCF must load TurtleBot3 and AWS clutter mesh assets."""
    mujoco = pytest.importorskip("mujoco")
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nmesh >= 6


def test_ppp_visual_place_detects_floor_contact_at_goal() -> None:
    goal = np.array([10.5, 2.8, 2.65])
    ee = np.array([10.5, 2.8, 0.59])
    assert rm._ppp_should_visually_place_cargo(
        ee,
        goal=goal,
        box_half_z=0.25,
        floor_z=0.25,
        goal_radius=0.5,
    )


def test_ppp_visual_place_not_triggered_during_transit() -> None:
    goal = np.array([10.5, 2.8, 2.65])
    ee = np.array([6.0, 2.0, 2.0])
    assert not rm._ppp_should_visually_place_cargo(
        ee,
        goal=goal,
        box_half_z=0.25,
        floor_z=0.25,
        goal_radius=0.5,
    )


def test_dubins_follow_distance_targets_car_fill() -> None:
    distance = rm._dubins_follow_distance(640, 720)
    assert 4.0 < distance < 18.0


def test_dubins_overview_distance_is_quarter_of_static() -> None:
    assert rm._DUBINS_OVERVIEW_DISTANCE_M == pytest.approx(
        rm._DUBINS_OVERVIEW_STATIC_DISTANCE_M * rm._DUBINS_OVERVIEW_DISTANCE_SCALE
    )
    assert 25.0 < rm._DUBINS_OVERVIEW_DISTANCE_M < 30.0


def test_dubins_race_midpoint_averages_agent_positions() -> None:
    rrt = np.array([10.0, 20.0, 0.0])
    sst = np.array([14.0, 24.0, 1.0])
    assert rm._dubins_race_midpoint(rrt, sst) == pytest.approx((12.0, 22.0, 0.4))


def test_make_dubins_overview_camera_targets_midpoint() -> None:
    mujoco = pytest.importorskip("mujoco")
    cam = rm._make_dubins_overview_camera(mujoco, (12.0, 22.0, 0.4))
    assert cam.lookat[0] == pytest.approx(12.0)
    assert cam.lookat[1] == pytest.approx(22.0)
    assert cam.distance == pytest.approx(rm._DUBINS_OVERVIEW_DISTANCE_M)
    assert cam.azimuth == pytest.approx(rm._DUBINS_OVERVIEW_AZIMUTH_DEG)
    assert cam.elevation == pytest.approx(rm._DUBINS_OVERVIEW_ELEVATION_DEG)


def test_simulate_dubins_race_poses_covers_transit() -> None:
    """Release dubins clip must traverse the warehouse floor."""
    pytest.importorskip("arco")
    rrt, sst, sim_time_s = rm.simulate_dubins_race_poses(
        "dubins_race",
        duration_s=None,
        fps=10,
    )
    assert rrt.shape[0] >= 2
    assert sst.shape[0] >= 2
    assert sim_time_s > 20.0
    assert rrt[:, 0].max() > 8.0
    assert sst[:, 0].max() > 8.0


def test_showcase_timing_real_time_factor() -> None:
    timing = rm.ShowcaseTiming(sim_time_s=10.0, render_duration_s=25.0)
    assert timing.real_time_factor == pytest.approx(2.5)


def test_resolve_scenario_duration_reads_yaml() -> None:
    assert rm.resolve_scenario_duration("ppp_warehouse") == pytest.approx(60.0)
    assert rm.resolve_scenario_duration("dubins_race") == pytest.approx(35.0)


# Mirrors .github/workflows/release.yml showcase render invocations.
_RELEASE_PPP_CLI: list[str] = [
    "--model",
    "ppp",
    "--scenario",
    "ppp_warehouse",
    "--all-cameras",
    "--collision-backend",
    "mujoco",
    "--planner-algorithm",
    "rrt_star",
    "--no-tracking",
    "--output-dir",
    "showcase_renders",
    "--timing-json",
    "showcase_renders/ppp_timing.json",
    "--full-duration",
    "--fps",
    "30",
    "--width",
    "1280",
    "--height",
    "720",
]

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
    "showcase_renders/dubins_timing.json",
    "--full-duration",
    "--fps",
    "30",
    "--width",
    "1280",
    "--height",
    "720",
]


def test_release_workflow_cli_args_parse() -> None:
    """Release workflow flags must parse without missing Namespace attributes."""
    parser = rm.build_parser()
    ppp_args = parser.parse_args(_RELEASE_PPP_CLI)
    dubins_args = parser.parse_args(_RELEASE_DUBINS_CLI)

    assert ppp_args.model == "ppp"
    assert ppp_args.scenario == "ppp_warehouse"
    assert ppp_args.all_cameras is True
    assert ppp_args.no_tracking is True
    assert ppp_args.timing_json == Path("showcase_renders/ppp_timing.json")
    assert ppp_args.no_realtime_postprocess is False
    assert ppp_args.full_duration is True

    assert dubins_args.model == "dubins"
    assert dubins_args.scenario == "dubins_race"
    assert dubins_args.all_cameras is True
    assert dubins_args.timing_json == Path("showcase_renders/dubins_timing.json")
    assert dubins_args.no_realtime_postprocess is False
    assert dubins_args.full_duration is True


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
    assert "usage:" in result.stdout.lower() or "usage:" in result.stderr.lower()


def test_main_accepts_release_ppp_cli(monkeypatch: pytest.MonkeyPatch) -> None:
    """main() must accept the exact release.yml PPP invocation."""
    timing = rm.ShowcaseTiming(sim_time_s=12.0, render_duration_s=12.0)

    def _fake_render(*_args: object, **_kwargs: object) -> list[rm.RenderResult]:
        return [
            rm.RenderResult(
                camera="overview",
                path=Path("/tmp/ppp_warehouse_overview.mp4"),
                frame_mean=42.0,
                timing=timing,
            )
        ]

    monkeypatch.setattr(rm, "render_showcase_videos", _fake_render)
    monkeypatch.setattr(rm, "postprocess_showcase_results", lambda r, **k: r)
    monkeypatch.setattr(rm, "write_showcase_timing_json", lambda *_a, **_k: None)

    assert rm.main(_RELEASE_PPP_CLI) == 0


def test_main_accepts_release_dubins_cli(monkeypatch: pytest.MonkeyPatch) -> None:
    """main() must accept the exact release.yml Dubins invocation."""
    timing = rm.ShowcaseTiming(sim_time_s=34.0, render_duration_s=34.0)

    def _fake_render(*_args: object, **_kwargs: object) -> list[rm.RenderResult]:
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
    monkeypatch.setattr(rm, "write_showcase_timing_json", lambda *_a, **_k: None)

    assert rm.main(_RELEASE_DUBINS_CLI) == 0
