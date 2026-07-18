"""Tests for fret.planning.PlannerNode."""

from __future__ import annotations

import numpy as np

from fret.planning.planner_node import PlannerNode
from fret.scene.occupancy_adapter import OccupancyAdapter


def test_construction(
    mock_occupancy_adapter: OccupancyAdapter,
    mock_kinematics: object,
    arm_planning_config: dict[str, object],
) -> None:
    PlannerNode(
        model="test_arm",
        occupancy_adapter=mock_occupancy_adapter,
        kinematics=mock_kinematics,
        planning_config=arm_planning_config,
    )


def test_invalid_config_returns_aborted(
    mock_occupancy_adapter: OccupancyAdapter,
    mock_kinematics: object,
    arm_planning_config: dict[str, object],
) -> None:
    """A goal outside joint limits must immediately return ABORTED."""
    from fret.interfaces import ErrorCode, PlanningRequest, PlanningStatus

    node = PlannerNode(
        model="test_arm",
        occupancy_adapter=mock_occupancy_adapter,
        kinematics=mock_kinematics,
        planning_config=arm_planning_config,
    )
    request = PlanningRequest(
        start_configuration=np.zeros(3),
        goal_configuration=np.array([999.0, 999.0, 999.0]),
        planning_timeout=5.0,
        scenario_id="test_invalid",
    )
    result = node.plan(request)  # type: ignore[attr-defined]
    assert result.status == PlanningStatus.ABORTED
    assert result.error_code == ErrorCode.INVALID_CONFIGURATION
    assert result.path == []


def test_valid_request_returns_success(
    mock_occupancy_adapter: OccupancyAdapter,
    mock_kinematics: object,
    arm_planning_config: dict[str, object],
) -> None:
    from fret.interfaces import PlanningRequest, PlanningStatus

    node = PlannerNode(
        model="test_arm",
        occupancy_adapter=mock_occupancy_adapter,
        kinematics=mock_kinematics,
        planning_config=arm_planning_config,
    )
    request = PlanningRequest(
        start_configuration=np.zeros(3),
        goal_configuration=np.array([0.3, 0.3, 0.05]),
        planning_timeout=30.0,
        scenario_id="test_valid",
    )
    result = node.plan(request)  # type: ignore[attr-defined]
    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2
