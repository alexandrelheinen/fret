"""E2E scenario test: Dubins race (SC-v11 / v1.1).

Pure-Python validation of SC-v11 acceptance criteria (no ROS required).
Runs in the default pytest suite (``--ignore=tests/integration``).
"""

from __future__ import annotations

import pathlib
import time
from collections.abc import Iterator

import numpy as np
import pytest

from fret.planning.dubins_obstacles import (
    default_obstacle_file,
    load_dubins_race_world,
)
from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import (
    SHOWCASE_PLANNER_RNG_SEED,
    deterministic_planner_rng,
)

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "dubins_race.yml"
)
_RACE_SIM_TIMEOUT_S = 240.0
# Wall clock includes dual planning plus real-TB3 race simulation.
_WALL_CLOCK_BUDGET_S = 300.0


@pytest.fixture(scope="module", autouse=True)
def _deterministic_planner_rng() -> Iterator[None]:
    """ARC planners use unseeded ``default_rng()``; pin for reproducible E2E."""
    with deterministic_planner_rng(SHOWCASE_PLANNER_RNG_SEED):
        yield


def test_obstacle_file_exists() -> None:
    assert default_obstacle_file().is_file()


def test_world_has_structure_forest() -> None:
    world = load_dubins_race_world()
    assert len(world.structures) >= 20


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
    assert result.rrt_time_to_goal_s <= _RACE_SIM_TIMEOUT_S
    assert result.sst_time_to_goal_s <= _RACE_SIM_TIMEOUT_S
    assert elapsed <= _WALL_CLOCK_BUDGET_S
    assert result.min_obstacle_clearance_m >= 0.0


def test_dummy_straight_line_collides_before_goal() -> None:
    """Grey dummy must fail on the naive diagonal before reaching goal B."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(record_poses=True)
    assert result.dummy_collided is True
    assert result.dummy_pose_history
    final = result.dummy_pose_history[-1]
    assert final[0] < 8.0
    assert final[1] < 8.0


def test_agents_can_take_different_path_lengths() -> None:
    """RRT* and SST must diverge through the structure forest (not one corridor)."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(record_poses=True)
    assert result.rrt_plan.path_found and result.sst_plan.path_found
    assert result.min_obstacle_clearance_m >= 0.0

    rrt_xy = np.array([(p[0], p[1]) for p in result.rrt_plan.path])
    sst_xy = np.array([(p[0], p[1]) for p in result.sst_plan.path])
    assert len(rrt_xy) >= 3
    assert len(sst_xy) >= 3

    n = 40
    t = np.linspace(0.0, 1.0, n)

    def _resample(path: np.ndarray) -> np.ndarray:
        cum = np.zeros(len(path))
        for i in range(1, len(path)):
            cum[i] = cum[i - 1] + np.linalg.norm(path[i] - path[i - 1])
        if cum[-1] <= 0.0:
            return np.repeat(path[:1], n, axis=0)
        samples = t * cum[-1]
        out = np.zeros((n, 2), dtype=np.float64)
        for i, s in enumerate(samples):
            j = min(int(np.searchsorted(cum, s)), len(path) - 1)
            out[i] = path[j]
        return out

    separation = np.linalg.norm(_resample(rrt_xy) - _resample(sst_xy), axis=1)
    # Compact 10 m lab: paths should still diverge by at least ~0.8 m.
    assert float(separation.max()) >= 0.8
