"""E2E scenario test: Dubins race (SC-v11 / v1.1).

ROS-free counterpart to ``tests/integration/test_scenario_dubins_race.py``.
Runs in the default pytest suite (``--ignore=tests/integration``).
"""

from __future__ import annotations

import pathlib
import time

from fret.planning.dubins_obstacles import (
    default_obstacle_file,
    load_dubins_race_world,
)
from fret.scenario.dubins_race_runner import DubinsRaceRunner

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "dubins_race.yml"
)
_RACE_TIMEOUT_S = 90.0


def test_obstacle_file_exists() -> None:
    assert default_obstacle_file().is_file()


def test_world_has_column_forest() -> None:
    world = load_dubins_race_world()
    assert len(world.columns) >= 100


def test_dual_agents_plan_and_finish_race() -> None:
    """V11-1: RRT* and SST plan independently and both reach goal B."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    t0 = time.monotonic()
    result = runner.run()
    elapsed = time.monotonic() - t0

    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.both_reached_goal is True
    assert result.rrt_time_to_goal_s is not None
    assert result.sst_time_to_goal_s is not None
    assert result.rrt_time_to_goal_s <= _RACE_TIMEOUT_S
    assert result.sst_time_to_goal_s <= _RACE_TIMEOUT_S
    assert elapsed <= _RACE_TIMEOUT_S + 30.0


def test_agents_can_take_different_path_lengths() -> None:
    """RRT* and SST must diverge through the column forest (not one corridor)."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(record_poses=True)
    assert result.rrt_plan.path_found and result.sst_plan.path_found

    rrt_path = result.rrt_plan.path
    sst_path = result.sst_plan.path
    assert len(rrt_path) >= 3
    assert len(sst_path) >= 3

    mid_rrt = rrt_path[len(rrt_path) // 2][:2]
    mid_sst = sst_path[len(sst_path) // 2][:2]
    mid_separation = float(
        (mid_rrt[0] - mid_sst[0]) ** 2 + (mid_rrt[1] - mid_sst[1]) ** 2
    ) ** 0.5
    assert mid_separation >= 5.0
