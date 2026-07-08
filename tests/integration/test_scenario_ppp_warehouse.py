"""End-to-end integration test: PPP warehouse scenario (SC-v10 / v1.0).

Validates releases.md acceptance criteria V10-2 – V10-5 in a deterministic,
ROS-free environment using ``PPPWarehouseRunner``.

Acceptance criteria checked:
  V10-2: ARCO RRT* finds a collision-free path within 30 s (MuJoCo contacts).
  V10-3: Gantry tracks trajectory; EE position error ≤ 10 mm.
  V10-4: Cargo welded at start zone, released at goal zone.
  V10-5: Planned path is collision-free (EE and welded cargo envelope).
"""

from __future__ import annotations

import pathlib
import time

import numpy as np
import pytest

from fret.interfaces import PlanningStatus
from fret.planning.ppp_obstacles import (
    load_ppp_warehouse_preview_obstacles,
    preview_obstacle_file,
)
from fret.scenario.ppp_warehouse_runner import PPPWarehouseRunner

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "ppp_warehouse.yml"
)
_PLANNING_TIMEOUT_S = 30.0
_EE_ERROR_LIMIT_M = 0.010


def test_preview_obstacle_file_exists() -> None:
    """Preview obstacle YAML must ship with the package."""
    assert preview_obstacle_file().is_file()


def test_preview_obstacles_match_mjcf_layout() -> None:
    """Preview layout must contain clutter boxes and back-wall shelf racks."""
    boxes = load_ppp_warehouse_preview_obstacles()
    assert len(boxes) == 5
    first = boxes[0]
    assert first.x_min == pytest.approx(3.2)
    assert first.x_max == pytest.approx(4.8)
    rack_b = boxes[4]
    assert rack_b.x_min == pytest.approx(7.04)
    assert rack_b.y_max == pytest.approx(4.0)


def test_planner_returns_success_within_timeout() -> None:
    """V10-2: planning must succeed within the scenario timeout."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    t0 = time.monotonic()
    result = runner.run()
    elapsed = time.monotonic() - t0
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.n_path_waypoints >= 2
    assert (
        elapsed <= _PLANNING_TIMEOUT_S
    ), f"E2E planning+run took {elapsed:.2f} s"


def test_max_tracking_error_within_10mm() -> None:
    """V10-3: controller tracking error must stay ≤ 10 mm."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run()
    assert result.planning_status == PlanningStatus.SUCCESS
    assert not result.controller_faulted
    assert result.max_tracking_error_m <= _EE_ERROR_LIMIT_M, (
        f"Max tracking error {result.max_tracking_error_m * 1000:.2f} mm "
        f"exceeds {_EE_ERROR_LIMIT_M * 1000:.0f} mm limit"
    )


def test_grasp_capture_and_release() -> None:
    """V10-4: magnetic grasp must weld at start and release at goal."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run()
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.grasp_captured is True
    assert result.grasp_released is True


def test_planned_path_collision_free() -> None:
    """V10-5: EE and welded cargo must avoid static preview obstacles."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run()
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.path_collision_free is True


def test_runner_loads_scenario_start_goal() -> None:
    """Runner must honour scenario start/goal configurations."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    params = runner.load_parameters()
    np.testing.assert_allclose(
        params["start_configuration"], [2.0, 1.0, 2.4], atol=1e-9
    )
    np.testing.assert_allclose(
        params["goal_configuration"], [10.5, 2.8, 2.65], atol=1e-9
    )
