"""Tests for scripts/render_mujoco.py (pure-Python helpers).

MuJoCo rendering is optional; these tests validate trajectory generation
and MJCF path resolution without a MuJoCo runtime.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# render_mujoco.py lives under scripts/, not src/
_SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
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
    assert cameras == [
        "overview",
        "aisle",
        "topdown",
        "follow",
        "pick",
    ]


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
    traj = rm.simulate_tracked_trajectory(
        dense,
        duration_s=30.0,
        fps=30,
        start=path[0],
    )
    assert traj.shape == (900, 3)
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
    traj = rm.build_showcase_trajectory(
        path,
        scenario="ppp_warehouse",
        duration_s=60.0,
        fps=30,
        collision_backend="mujoco",
        use_tracking=False,
    )
    assert traj.shape == (1800, 3)
    assert traj[:, 0].max() > 8.0
    assert traj[:, 1].max() > 2.0


def test_list_showcase_cameras_dubins_race() -> None:
    path = rm.resolve_mjcf_path("dubins", "dubins_race", None)
    cameras = rm.list_showcase_cameras(path, scenario="dubins_race")
    assert cameras == ["overview", "topdown", "follow", "finish"]


def test_simulate_dubins_race_poses_covers_transit() -> None:
    """Release dubins clip must traverse the warehouse floor."""
    pytest.importorskip("arco")
    rrt, sst = rm.simulate_dubins_race_poses(
        "dubins_race",
        duration_s=20.0,
        fps=10,
    )
    assert rrt.shape[0] == 200
    assert sst.shape[0] == 200
    assert rrt[:, 0].max() > 8.0
    assert sst[:, 0].max() > 8.0


def test_resolve_scenario_duration_reads_yaml() -> None:
    assert rm.resolve_scenario_duration("ppp_warehouse") == pytest.approx(60.0)
    assert rm.resolve_scenario_duration("dubins_race") == pytest.approx(35.0)
