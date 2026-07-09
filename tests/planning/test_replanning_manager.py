"""Tests for fret.planning.ReplanningManager.

Acceptance criteria (FR-PLN-09):
  - Correct initial IDLE state.
  - start_execution returns TrajectoryResult on success.
  - trigger_replan debounces within min_replan_interval.
  - Tracking error / occupancy thresholds correctly gate replanning.
  - Max replan attempts transitions to HALTED.
  - halt() and reset() work as expected.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from fret.interfaces import (
    ErrorCode,
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningResult,
    PlanningStatus,
)
from fret.config_loader import load_algorithm_config
from fret.planning.replanning_manager import (
    ManagerState,
    ReplanningManager,
    TriggerKind,
)
from fret.planning.trajectory_converter import (
    TrajectoryConverter,
    TrajectoryResult,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

_DOF = 3
_START = np.zeros(_DOF)
_GOAL = np.array([0.3, 0.3, 0.05])

_SUCCESS_PATH = [_START.copy(), _GOAL.copy()]
_SUCCESS_RESULT = PlanningResult(
    status=PlanningStatus.SUCCESS,
    path=_SUCCESS_PATH,
    error_code=ErrorCode.NONE,
    planning_duration=0.1,
)
_FAILURE_RESULT = PlanningResult(
    status=PlanningStatus.ABORTED,
    path=[],
    error_code=ErrorCode.INVALID_CONFIGURATION,
    planning_duration=0.01,
)


def _make_request() -> PlanningRequest:
    return PlanningRequest(
        start_configuration=_START.copy(),
        goal_configuration=_GOAL.copy(),
        planning_timeout=5.0,
        scenario_id="test",
    )


def _make_planner(success: bool = True) -> MagicMock:
    planner = MagicMock()
    planner.plan.return_value = _SUCCESS_RESULT if success else _FAILURE_RESULT
    return planner


def _make_converter() -> MagicMock:
    converter = MagicMock(spec=TrajectoryConverter)
    traj = TrajectoryResult(
        joint_names=["joint_arm_0", "joint_arm_1", "joint_extension"],
        positions=[_START.copy(), _GOAL.copy()],
        velocities=[np.zeros(_DOF), np.zeros(_DOF)],
        timestamps=[0.0, 1.0],
        duration=1.0,
    )
    converter.convert.return_value = traj
    return converter


def _default_replanning_config() -> dict[str, object]:
    return load_algorithm_config("planning/scara.yml")


def _replanning_config(**overrides: object) -> dict[str, object]:
    base = dict(_default_replanning_config()["replanning"])  # type: ignore[index]
    base.update(overrides)
    return {"replanning": base}


def _make_manager(
    success: bool = True,
    config: dict | None = None,
) -> ReplanningManager:
    return ReplanningManager(
        planner_node=_make_planner(success),
        trajectory_converter=_make_converter(),
        config=config if config is not None else _default_replanning_config(),
    )


def _empty_payload() -> OccupancyUpdatePayload:
    return OccupancyUpdatePayload(
        obstacle_points=np.zeros((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )


def _payload_with_points(pts: np.ndarray) -> OccupancyUpdatePayload:
    return OccupancyUpdatePayload(
        obstacle_points=pts,
        timestamp=time.time(),
        frame_id="world",
    )


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


def test_construction() -> None:
    """No crash on construction."""
    _make_manager()


def test_initial_state_idle() -> None:
    """State is IDLE after construction."""
    mgr = _make_manager()
    assert mgr.state == ManagerState.IDLE


# ---------------------------------------------------------------------------
# start_execution
# ---------------------------------------------------------------------------


def test_start_execution_success() -> None:
    """Returns TrajectoryResult and transitions to EXECUTING on success."""
    mgr = _make_manager(success=True)
    result = mgr.start_execution(_make_request())
    assert isinstance(result, TrajectoryResult)
    assert mgr.state == ManagerState.EXECUTING


def test_start_execution_failure() -> None:
    """Returns None and stays non-EXECUTING when planning fails."""
    mgr = _make_manager(success=False)
    result = mgr.start_execution(_make_request())
    assert result is None
    assert mgr.state != ManagerState.EXECUTING


def test_trajectory_result_used() -> None:
    """Result of start_execution is a TrajectoryResult instance."""
    mgr = _make_manager()
    result = mgr.start_execution(_make_request())
    assert isinstance(result, TrajectoryResult)


# ---------------------------------------------------------------------------
# trigger_replan
# ---------------------------------------------------------------------------


def test_trigger_replan_from_executing() -> None:
    """Trigger while EXECUTING returns True."""
    mgr = _make_manager()
    mgr.start_execution(_make_request())
    assert mgr.state == ManagerState.EXECUTING
    ok = mgr.trigger_replan(TriggerKind.MANUAL)
    assert ok is True


def test_trigger_replan_when_idle_returns_false() -> None:
    """Cannot trigger replan when not in EXECUTING state."""
    mgr = _make_manager()
    assert mgr.state == ManagerState.IDLE
    assert mgr.trigger_replan(TriggerKind.MANUAL) is False


def test_trigger_replan_debounce() -> None:
    """Second trigger within min_interval returns False."""
    mgr = ReplanningManager(
        planner_node=_make_planner(),
        trajectory_converter=_make_converter(),
        config=_replanning_config(min_replan_interval=60.0),
    )
    mgr.start_execution(_make_request())
    first = mgr.trigger_replan(TriggerKind.MANUAL)
    assert first is True
    second = mgr.trigger_replan(TriggerKind.MANUAL)
    assert second is False


# ---------------------------------------------------------------------------
# Tracking error
# ---------------------------------------------------------------------------


def test_tracking_error_below_threshold_no_replan() -> None:
    """Small error does not trigger replan."""
    mgr = _make_manager(
        config=_replanning_config(tracking_error_threshold=0.02)
    )
    mgr.start_execution(_make_request())
    # Manually reset last replan time so debounce is not active.
    mgr._last_replan_time = -float("inf")
    initial_attempts = mgr._replan_attempts
    mgr.report_tracking_error(0.005)
    assert mgr._replan_attempts == initial_attempts


def test_tracking_error_above_threshold_triggers_replan() -> None:
    """Large error triggers replan."""
    mgr = _make_manager(
        config=_replanning_config(tracking_error_threshold=0.02)
    )
    mgr.start_execution(_make_request())
    mgr._last_replan_time = -float("inf")
    initial_attempts = mgr._replan_attempts
    mgr.report_tracking_error(0.05)
    assert mgr._replan_attempts > initial_attempts


# ---------------------------------------------------------------------------
# Occupancy updates
# ---------------------------------------------------------------------------


def test_occupancy_update_small_change_no_replan() -> None:
    """Tiny obstacle displacement does not trigger replan."""
    mgr = _make_manager(
        config=_replanning_config(occupancy_change_threshold=0.05)
    )
    mgr.start_execution(_make_request())
    pts = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.5]], dtype=np.float64)
    mgr.report_occupancy_update(_payload_with_points(pts))
    initial_attempts = mgr._replan_attempts
    mgr._last_replan_time = -float("inf")
    # Tiny displacement (< threshold).
    pts2 = pts + 0.001
    mgr.report_occupancy_update(_payload_with_points(pts2))
    assert mgr._replan_attempts == initial_attempts


def test_occupancy_update_large_change_triggers_replan() -> None:
    """Large obstacle displacement triggers replan."""
    mgr = _make_manager(
        config=_replanning_config(occupancy_change_threshold=0.05)
    )
    mgr.start_execution(_make_request())
    pts = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.5]], dtype=np.float64)
    mgr.report_occupancy_update(_payload_with_points(pts))
    mgr._last_replan_time = -float("inf")
    initial_attempts = mgr._replan_attempts
    pts2 = pts + 0.5  # large shift
    mgr.report_occupancy_update(_payload_with_points(pts2))
    assert mgr._replan_attempts > initial_attempts


def test_report_occupancy_updates_adapter() -> None:
    """Occupancy payload is stored after report_occupancy_update."""
    mgr = _make_manager()
    payload = _empty_payload()
    mgr.report_occupancy_update(payload)
    assert mgr._last_payload is payload


# ---------------------------------------------------------------------------
# halt / reset
# ---------------------------------------------------------------------------


def test_halt_transitions_to_halted() -> None:
    """halt() transitions to HALTED."""
    mgr = _make_manager()
    mgr.halt()
    assert mgr.state == ManagerState.HALTED


def test_reset_from_halted() -> None:
    """reset() from HALTED returns to IDLE."""
    mgr = _make_manager()
    mgr.halt()
    mgr.reset()
    assert mgr.state == ManagerState.IDLE


def test_reset_from_executing() -> None:
    """reset() from EXECUTING returns to IDLE."""
    mgr = _make_manager()
    mgr.start_execution(_make_request())
    mgr.reset()
    assert mgr.state == ManagerState.IDLE


# ---------------------------------------------------------------------------
# Max replan attempts
# ---------------------------------------------------------------------------


def test_max_replan_attempts_halts() -> None:
    """Exceeding max_replan_attempts transitions to HALTED."""
    mgr = ReplanningManager(
        planner_node=_make_planner(success=True),
        trajectory_converter=_make_converter(),
        config=_replanning_config(max_replan_attempts=2, min_replan_interval=0.0),
    )
    mgr.start_execution(_make_request())
    # Burn through all attempts.
    for _ in range(5):
        mgr._last_replan_time = -float("inf")
        mgr.trigger_replan(TriggerKind.MANUAL)
        if mgr.state == ManagerState.HALTED:
            break
    assert mgr.state == ManagerState.HALTED


# ---------------------------------------------------------------------------
# Enum values
# ---------------------------------------------------------------------------


def test_manager_state_enum_values() -> None:
    """ManagerState enum has the expected string values."""
    assert ManagerState.IDLE.value == "IDLE"
    assert ManagerState.EXECUTING.value == "EXECUTING"
    assert ManagerState.REPLANNING.value == "REPLANNING"
    assert ManagerState.HALTED.value == "HALTED"


def test_trigger_kind_enum_values() -> None:
    """TriggerKind enum has the expected string values."""
    assert TriggerKind.OCCUPANCY_CHANGE.value == "OCCUPANCY_CHANGE"
    assert TriggerKind.PATH_INVALIDATED.value == "PATH_INVALIDATED"
    assert TriggerKind.TRACKING_ERROR.value == "TRACKING_ERROR"
    assert TriggerKind.GOAL_UPDATE.value == "GOAL_UPDATE"
    assert TriggerKind.MANUAL.value == "MANUAL"


# ---------------------------------------------------------------------------
# Trajectory after replan
# ---------------------------------------------------------------------------


def test_replan_updates_trajectory() -> None:
    """Trajectory changes (converter called again) after successful replan."""
    converter = _make_converter()
    mgr = ReplanningManager(
        planner_node=_make_planner(success=True),
        trajectory_converter=converter,
        config=_replanning_config(min_replan_interval=0.0),
    )
    mgr.start_execution(_make_request())
    call_count_before = converter.convert.call_count
    mgr._last_replan_time = -float("inf")
    mgr.trigger_replan(TriggerKind.MANUAL)
    assert converter.convert.call_count > call_count_before
