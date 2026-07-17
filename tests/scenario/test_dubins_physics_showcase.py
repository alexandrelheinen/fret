"""Regression tests for Dubins physics SITL: clearance and wheel kinematics."""

from __future__ import annotations

import math
from collections.abc import Iterator

import numpy as np
import pytest

from fret.control.dubins_wheel_model import max_body_lateral_speed_m_s
from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import (
    SHOWCASE_PLANNER_RNG_SEED,
    deterministic_planner_rng,
)

_GOAL_XY = (8.8, 8.8)
_GOAL_RADIUS_M = 0.35
_MAX_PHYSICS_LATERAL_SPEED_M_S = 0.22


@pytest.fixture(scope="module", autouse=True)
def _seeded_planner_rng() -> Iterator[None]:
    with deterministic_planner_rng(SHOWCASE_PLANNER_RNG_SEED):
        yield


def _distance_to_goal(pose: tuple[float, float, float]) -> float:
    return float(math.hypot(pose[0] - _GOAL_XY[0], pose[1] - _GOAL_XY[1]))


def test_physics_pose_history_non_negative_clearance() -> None:
    """No agent body samples may penetrate structure occupancy (green SST included)."""
    result = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=True,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )
    assert result.both_reached_goal is True
    assert result.min_obstacle_clearance_m >= 0.0


def test_physics_agents_finish_without_lateral_skid() -> None:
    """True wheel agents must not produce sustained body-lateral skid."""
    result = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=True,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )
    dt = 0.05
    rrt_lat = max_body_lateral_speed_m_s(
        result.rrt_pose_history, dt=dt, max_yaw_rate_rad_s=1.0
    )
    sst_lat = max_body_lateral_speed_m_s(
        result.sst_pose_history, dt=dt, max_yaw_rate_rad_s=1.0
    )
    assert rrt_lat <= _MAX_PHYSICS_LATERAL_SPEED_M_S
    assert sst_lat <= _MAX_PHYSICS_LATERAL_SPEED_M_S


def test_kinematic_showcase_export_reaches_goal_for_both_agents() -> None:
    """Release kinematic clips must end with both agents inside goal radius."""
    pytest.importorskip("mujoco")
    import scripts.render_mujoco as rm

    rrt, sst, _dummy, _sim_time_s = rm.simulate_dubins_race_poses(
        "dubins_race",
        duration_s=None,
        fps=30,
        physics_mode=False,
    )
    assert _distance_to_goal(tuple(rrt[-1])) <= _GOAL_RADIUS_M
    assert _distance_to_goal(tuple(sst[-1])) <= _GOAL_RADIUS_M
