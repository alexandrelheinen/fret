"""Prove joint-space MPC is wired with C-space obstacle barriers."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from fret.control.cspace_mpc_occupancy import (
    build_wall_cspace_barrier_occupancy,
)
from fret.control.joint_mpc import JointPathMPCTracker, build_omx_joint_mpc
from fret.control.kinematics import Kinematics
from fret.control.pick_place_clutter_sim import (
    _barrier_occupancy_for_scenario,
    _joint_mpc_for_model,
    _require_mpc_occupancy,
)
from fret.mjcf.omx import ensure_omx_wall_maze_mjcf
from fret.sitl_config import load_scenario_parameters

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/omx_wall_maze.yml")


def test_wall_maze_builds_nonzero_cspace_occupancy() -> None:
    params = load_scenario_parameters(_SCENARIO)
    occ = _barrier_occupancy_for_scenario(
        params,
        robot_model="open_manipulator_x",
        scenario_id="omx_wall_maze",
        seed=204,
    )
    assert occ is not None
    pts = np.asarray(occ.points, dtype=np.float64)
    real = pts[np.linalg.norm(pts, axis=1) < 50.0]
    assert real.shape[0] >= 50
    assert float(occ.clearance) >= 0.2


def test_wall_maze_joint_mpc_has_occupancy_attached() -> None:
    params = load_scenario_parameters(_SCENARIO)
    occ = _barrier_occupancy_for_scenario(
        params,
        robot_model="open_manipulator_x",
        scenario_id="omx_wall_maze",
        seed=204,
    )
    mpc = _joint_mpc_for_model(
        "open_manipulator_x", occupancy=occ, params=params
    )
    _require_mpc_occupancy(mpc, context="test")
    assert mpc._occ is occ
    assert float(mpc.config.weight_obstacle) >= 100.0


def test_require_mpc_occupancy_rejects_bare_tracker() -> None:
    mpc = build_omx_joint_mpc()
    with pytest.raises(RuntimeError, match="occupancy=None"):
        _require_mpc_occupancy(mpc, context="bare")


def test_path_tracker_with_occupancy_avoids_wall_target() -> None:
    """Carrot MPC with barriers must not dive onto a wall-collision waypoint."""
    kin = Kinematics("open_manipulator_x")
    xml = ensure_omx_wall_maze_mjcf()
    occ = build_wall_cspace_barrier_occupancy(
        mjcf_path=str(xml),
        joint_limits=kin.joint_limits,
        clearance=0.28,
        n_samples=12000,
        rng=np.random.default_rng(9),
    )
    pts = np.asarray(occ.points, dtype=np.float64)
    pts = pts[np.linalg.norm(pts, axis=1) < 50.0]
    assert pts.shape[0] >= 30
    target = pts[0].copy()
    rng = np.random.default_rng(13)
    lo = np.asarray([lim[0] for lim in kin.joint_limits], dtype=np.float64)
    hi = np.asarray([lim[1] for lim in kin.joint_limits], dtype=np.float64)
    start = None
    for _ in range(8000):
        cand = np.clip(target + rng.normal(0.0, 0.4, size=4), lo, hi)
        if (
            not occ.is_occupied(cand)
            and float(np.linalg.norm(cand - target)) > 0.4
        ):
            start = cand
            break
    assert start is not None
    # Dense path that ends inside the wall cloud — open tracker follows it in.
    path = [
        start,
        0.67 * start + 0.33 * target,
        0.33 * start + 0.67 * target,
        target,
    ]

    open_tracker = JointPathMPCTracker(
        path,
        build_omx_joint_mpc(),
        race_speed=1.5,
        max_carrot_lag=0.55,
        goal_tol=0.08,
    )
    open_tracker.reset(start)
    min_open = 1.0e9
    for _ in range(70):
        q = open_tracker.step(0.05)
        d, _ = occ.nearest_obstacle(q)
        min_open = min(min_open, float(d))

    from arco.control.mpc import JointSpaceMPCConfig

    safe_cfg = replace(
        JointSpaceMPCConfig.create_from_config(), weight_obstacle=120.0
    )
    safe_tracker = JointPathMPCTracker(
        path,
        build_omx_joint_mpc(occupancy=occ, mpc_cfg=safe_cfg),
        race_speed=1.5,
        max_carrot_lag=0.55,
        goal_tol=0.08,
    )
    safe_tracker.reset(start)
    min_safe = 1.0e9
    for _ in range(70):
        q = safe_tracker.step(0.05)
        d, _ = occ.nearest_obstacle(q)
        min_safe = min(min_safe, float(d))

    assert min_open < 0.05, "sanity: open path tracker enters wall cloud"
    assert min_safe > min_open + 0.05
    assert min_safe > 0.10
