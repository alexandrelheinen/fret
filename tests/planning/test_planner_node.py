"""Tests for fret.planning.PlannerNode.

Acceptance criteria (FR-PLN-01 through FR-PLN-07):
  - Invalid goal configuration → ABORTED + INVALID_CONFIGURATION.
  - Planning timeout exhausted → ABORTED + TIMEOUT.
  - Successful planning → SUCCESS with a non-empty path and no error.
  - Post-processing failure → ABORTED + POST_PROCESS_FAILED.
  - Cancelled goal → CANCELLED + NONE.
"""

from __future__ import annotations

import pytest

from fret.planning.planner_node import PlannerNode
from fret.scene.occupancy_adapter import OccupancyAdapter


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction(mock_occupancy_adapter: OccupancyAdapter) -> None:
    PlannerNode(model="scara", occupancy_adapter=mock_occupancy_adapter)


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_invalid_config_returns_aborted(
    mock_occupancy_adapter: OccupancyAdapter,
) -> None:
    """A goal outside joint limits must immediately return ABORTED /
    INVALID_CONFIGURATION without calling the ARCO planner.
    """
    from fret.interfaces import ErrorCode, PlanningRequest, PlanningStatus

    import numpy as np

    node = PlannerNode(model="scara", occupancy_adapter=mock_occupancy_adapter)
    request = PlanningRequest(
        start_configuration=np.zeros(3),
        goal_configuration=np.array([999.0, 999.0, 999.0]),  # out of limits
        planning_timeout=5.0,
        scenario_id="test_invalid",
    )
    result = node.plan(request)  # type: ignore[attr-defined]
    assert result.status == PlanningStatus.ABORTED
    assert result.error_code == ErrorCode.INVALID_CONFIGURATION
    assert result.path == []


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_valid_request_returns_success(
    mock_occupancy_adapter: OccupancyAdapter,
) -> None:
    from fret.interfaces import PlanningRequest, PlanningStatus

    import numpy as np

    node = PlannerNode(model="scara", occupancy_adapter=mock_occupancy_adapter)
    request = PlanningRequest(
        start_configuration=np.zeros(3),
        goal_configuration=np.array([0.3, 0.3, 0.05]),
        planning_timeout=30.0,
        scenario_id="test_valid",
    )
    result = node.plan(request)  # type: ignore[attr-defined]
    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2
