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


def test_cspace_occupancy_answers_the_full_arco_map_protocol() -> None:
    """Compiled ARCO planners call ``segment_free`` on the map they get.

    ARCO v0.5.0 runs RRT*, SST and ``TrajectoryPruner`` as Rust, and they
    reach a Python map through the ``Occupancy`` base rather than through
    ``is_occupied`` alone, so the adapter has to inherit that base.
    """
    from arco.mapping.occupancy import Occupancy

    from fret.planning.planner_node import _CSpaceOccupancy

    class _BlockedBand:
        """Collision-free everywhere except a band around x = 0.5."""

        @staticmethod
        def is_collision_free(q: np.ndarray) -> bool:
            return not 0.4 <= float(q[0]) <= 0.6

    occupancy = _CSpaceOccupancy(_BlockedBand())
    assert isinstance(occupancy, Occupancy)

    free = np.array([0.0, 0.0], dtype=np.float64)
    blocked = np.array([0.5, 0.0], dtype=np.float64)
    beyond = np.array([1.0, 0.0], dtype=np.float64)

    assert occupancy.is_occupied(blocked) is True
    assert occupancy.is_occupied(free) is False
    assert occupancy.segment_free(free, beyond) is False
    assert occupancy.segment_free(free, np.array([0.3, 0.0])) is True
    assert occupancy.nearest_obstacle(blocked)[0] == 0.0
    assert occupancy.nearest_obstacle(free)[0] == float("inf")
