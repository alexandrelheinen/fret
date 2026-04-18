"""Tests for the fret.interfaces module.

These tests exercise the public data contracts defined in
``interfaces/enums.py`` and ``interfaces/types.py``.  The enums and dataclass
invariants are pure Python — no stubs involved — so all tests must pass
unconditionally.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.interfaces import (
    ErrorCode,
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningResult,
    PlanningStatus,
    RobotState,
)

# ---------------------------------------------------------------------------
# PlanningStatus
# ---------------------------------------------------------------------------


class TestPlanningStatus:
    def test_values_are_ints(self) -> None:
        assert int(PlanningStatus.SUCCESS) == 0
        assert int(PlanningStatus.ABORTED) == 1
        assert int(PlanningStatus.CANCELLED) == 2

    def test_three_members(self) -> None:
        assert len(PlanningStatus) == 3


# ---------------------------------------------------------------------------
# ErrorCode
# ---------------------------------------------------------------------------


class TestErrorCode:
    def test_none_is_zero(self) -> None:
        assert int(ErrorCode.NONE) == 0

    def test_internal_error_is_99(self) -> None:
        assert int(ErrorCode.INTERNAL_ERROR) == 99

    def test_all_codes_unique(self) -> None:
        values = [int(e) for e in ErrorCode]
        assert len(values) == len(set(values))


# ---------------------------------------------------------------------------
# OccupancyUpdatePayload
# ---------------------------------------------------------------------------


class TestOccupancyUpdatePayload:
    def test_valid_construction(self) -> None:
        pts = np.zeros((5, 3), dtype=np.float64)
        payload = OccupancyUpdatePayload(
            obstacle_points=pts, timestamp=1.0, frame_id="world"
        )
        assert payload.frame_id == "world"
        assert payload.obstacle_points.shape == (5, 3)

    def test_empty_points_valid(self) -> None:
        pts = np.empty((0, 3), dtype=np.float64)
        payload = OccupancyUpdatePayload(
            obstacle_points=pts, timestamp=0.0, frame_id="world"
        )
        assert payload.obstacle_points.shape == (0, 3)

    def test_wrong_frame_id_raises(self) -> None:
        pts = np.zeros((3, 3))
        with pytest.raises(ValueError, match="frame_id"):
            OccupancyUpdatePayload(
                obstacle_points=pts, timestamp=0.0, frame_id="base_link"
            )

    def test_wrong_shape_raises(self) -> None:
        pts = np.zeros((3, 2))  # wrong last dimension
        with pytest.raises(ValueError, match="shape"):
            OccupancyUpdatePayload(
                obstacle_points=pts, timestamp=0.0, frame_id="world"
            )


# ---------------------------------------------------------------------------
# RobotState
# ---------------------------------------------------------------------------


class TestRobotState:
    def test_valid_scara(self) -> None:
        state = RobotState(
            joint_positions=np.zeros(3),
            joint_velocities=np.zeros(3),
            joint_names=["joint_1", "joint_2", "joint_3"],
            timestamp=0.0,
        )
        assert len(state.joint_names) == 3

    def test_position_shape_mismatch_raises(self) -> None:
        with pytest.raises(ValueError, match="joint_positions"):
            RobotState(
                joint_positions=np.zeros(2),  # wrong DOF
                joint_velocities=np.zeros(3),
                joint_names=["j1", "j2", "j3"],
                timestamp=0.0,
            )

    def test_velocity_shape_mismatch_raises(self) -> None:
        with pytest.raises(ValueError, match="joint_velocities"):
            RobotState(
                joint_positions=np.zeros(3),
                joint_velocities=np.zeros(2),  # wrong DOF
                joint_names=["j1", "j2", "j3"],
                timestamp=0.0,
            )


# ---------------------------------------------------------------------------
# PlanningRequest
# ---------------------------------------------------------------------------


class TestPlanningRequest:
    def test_valid(self) -> None:
        req = PlanningRequest(
            start_configuration=np.zeros(3),
            goal_configuration=np.array([0.5, 0.3, 0.1]),
            planning_timeout=10.0,
            scenario_id="static_reach",
        )
        assert req.planning_timeout == 10.0

    def test_zero_timeout_raises(self) -> None:
        with pytest.raises(ValueError, match="planning_timeout"):
            PlanningRequest(
                start_configuration=np.zeros(3),
                goal_configuration=np.zeros(3),
                planning_timeout=0.0,
                scenario_id="test",
            )

    def test_shape_mismatch_raises(self) -> None:
        with pytest.raises(ValueError, match="same shape"):
            PlanningRequest(
                start_configuration=np.zeros(3),
                goal_configuration=np.zeros(4),
                planning_timeout=5.0,
                scenario_id="test",
            )


# ---------------------------------------------------------------------------
# PlanningResult
# ---------------------------------------------------------------------------


class TestPlanningResult:
    def test_success_requires_path(self) -> None:
        with pytest.raises(ValueError, match="at least 2 waypoints"):
            PlanningResult(
                status=PlanningStatus.SUCCESS,
                path=[np.zeros(3)],  # only 1 waypoint
                error_code=ErrorCode.NONE,
            )

    def test_success_valid(self) -> None:
        result = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[np.zeros(3), np.ones(3)],
            error_code=ErrorCode.NONE,
            planning_duration=1.5,
            iteration_count=42,
        )
        assert result.status == PlanningStatus.SUCCESS

    def test_aborted_empty_path(self) -> None:
        result = PlanningResult(
            status=PlanningStatus.ABORTED,
            path=[],
            error_code=ErrorCode.TIMEOUT,
        )
        assert result.error_code == ErrorCode.TIMEOUT

    def test_aborted_nonempty_path_raises(self) -> None:
        with pytest.raises(ValueError, match="empty path"):
            PlanningResult(
                status=PlanningStatus.ABORTED,
                path=[np.zeros(3)],
                error_code=ErrorCode.TIMEOUT,
            )

    def test_success_non_none_error_raises(self) -> None:
        with pytest.raises(ValueError, match="error_code NONE"):
            PlanningResult(
                status=PlanningStatus.SUCCESS,
                path=[np.zeros(3), np.ones(3)],
                error_code=ErrorCode.TIMEOUT,
            )
