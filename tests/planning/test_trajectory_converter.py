"""Tests for fret.planning.TrajectoryConverter.

Acceptance criteria (FR-PLN-08):
  - Converts a raw joint-space path into a time-parameterized trajectory.
  - Result has at least 2 positions with monotonically increasing timestamps.
  - Velocities array has the same shape as positions array.
  - Respects per-joint velocity limits (approximately).
  - Raises ValueError for a path shorter than 2 waypoints.
  - start_time offset is correctly applied to all timestamps.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.config_loader import load_algorithm_config
from fret.planning.trajectory_converter import (
    TrajectoryConverter,
    TrajectoryResult,
)

_SCARA_PLANNING = load_algorithm_config("planning/scara.yml")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_DOF = 3
_START = np.zeros(_DOF)
_GOAL = np.array([0.5, 0.5, 0.1])


def _make_converter(**kwargs: object) -> TrajectoryConverter:
    cfg = dict(_SCARA_PLANNING)
    if kwargs:
        cfg = {**cfg, **kwargs}
    return TrajectoryConverter(config=cfg)


def _two_waypoint_result(
    converter: TrajectoryConverter | None = None,
) -> TrajectoryResult:
    c = converter or TrajectoryConverter(config=_SCARA_PLANNING)
    return c.convert([_START.copy(), _GOAL.copy()])


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


def test_construction_default() -> None:
    """No crash with bundled SCARA configuration."""
    TrajectoryConverter(config=_SCARA_PLANNING)


def test_construction_with_config() -> None:
    """Custom config values are applied."""
    cfg = {
        "trajectory": {
            **_SCARA_PLANNING["trajectory"],  # type: ignore[index]
            "control_hz": 10.0,
            "v_max": [1.0, 1.0, 0.05],
            "a_max": [1.5, 1.5, 0.1],
            "dt_min": 0.05,
        }
    }
    c = TrajectoryConverter(config=cfg)
    # Trigger conversion to verify config was consumed without error.
    result = c.convert([_START.copy(), _GOAL.copy()])
    assert result is not None


def test_construction_nested_config() -> None:
    """Config under a 'trajectory' sub-key is accepted."""
    cfg = {
        "trajectory": {
            **_SCARA_PLANNING["trajectory"],  # type: ignore[index]
            "control_hz": 20.0,
        }
    }
    TrajectoryConverter(config=cfg)


# ---------------------------------------------------------------------------
# Basic conversion
# ---------------------------------------------------------------------------


def test_two_waypoint_path() -> None:
    """Converts 2-waypoint path; result has >= 2 positions."""
    result = _two_waypoint_result()
    assert len(result.positions) >= 2


def test_positions_start_and_end() -> None:
    """First and last positions match path start and goal."""
    result = _two_waypoint_result()
    np.testing.assert_allclose(result.positions[0], _START, atol=1e-9)
    np.testing.assert_allclose(result.positions[-1], _GOAL, atol=1e-9)


def test_timestamps_monotonic() -> None:
    """Timestamps must be strictly increasing."""
    result = _two_waypoint_result()
    for i in range(1, len(result.timestamps)):
        assert (
            result.timestamps[i] > result.timestamps[i - 1]
        ), f"timestamp[{i}]={result.timestamps[i]} not > timestamp[{i - 1}]={result.timestamps[i - 1]}"


def test_velocities_shape() -> None:
    """Velocities list has the same length as positions; each element is (DOF,)."""
    result = _two_waypoint_result()
    assert len(result.velocities) == len(result.positions)
    for v in result.velocities:
        assert v.shape == (_DOF,)


def test_joint_names() -> None:
    """Correct SCARA joint names are returned."""
    result = _two_waypoint_result()
    assert result.joint_names == [
        "joint_arm_0",
        "joint_arm_1",
        "joint_extension",
    ]


def test_duration_positive() -> None:
    """Duration is strictly positive for a non-trivial path."""
    result = _two_waypoint_result()
    assert result.duration > 0.0


def test_returns_trajectory_result_type() -> None:
    """Return value is a TrajectoryResult instance."""
    result = _two_waypoint_result()
    assert isinstance(result, TrajectoryResult)


# ---------------------------------------------------------------------------
# Multi-segment path
# ---------------------------------------------------------------------------


def test_multi_segment_path() -> None:
    """5-waypoint path produces a valid result."""
    path = [
        np.zeros(_DOF),
        np.array([0.2, 0.1, 0.02]),
        np.array([0.4, 0.2, 0.05]),
        np.array([0.3, 0.4, 0.08]),
        np.array([0.5, 0.5, 0.10]),
    ]
    result = TrajectoryConverter(config=_SCARA_PLANNING).convert(path)
    assert len(result.positions) >= 2
    np.testing.assert_allclose(result.positions[0], path[0], atol=1e-9)
    np.testing.assert_allclose(result.positions[-1], path[-1], atol=1e-9)


# ---------------------------------------------------------------------------
# Zero-motion path
# ---------------------------------------------------------------------------


def test_zero_motion_path() -> None:
    """start == goal → still returns >= 2 points and duration >= dt_min."""
    c = TrajectoryConverter(config=_SCARA_PLANNING)
    result = c.convert([_START.copy(), _START.copy()])
    assert len(result.positions) >= 2
    assert result.duration >= 0.02  # dt_min default


# ---------------------------------------------------------------------------
# Velocity limits
# ---------------------------------------------------------------------------


def test_v_max_respected() -> None:
    """Max velocity in output does not significantly exceed per-joint v_max."""
    c = TrajectoryConverter(config=_SCARA_PLANNING)
    result = c.convert([_START.copy(), _GOAL.copy()])
    v_max = np.array([1.5, 1.5, 0.1])
    for vel in result.velocities:
        # Allow 10 % tolerance for finite-difference approximation at edges.
        np.testing.assert_array_less(np.abs(vel), v_max * 1.1 + 1e-6)


# ---------------------------------------------------------------------------
# Control rate
# ---------------------------------------------------------------------------


def test_control_hz_sets_rate() -> None:
    """At 10 Hz for a ~1 s path, the sample count is approximately 10."""
    cfg = {
        "trajectory": {
            **_SCARA_PLANNING["trajectory"],  # type: ignore[index]
            "control_hz": 10.0,
        }
    }
    c = TrajectoryConverter(config=cfg)
    # Create path long enough for ~1s duration given default v_max.
    path = [np.zeros(_DOF), np.array([1.5, 0.0, 0.0])]
    result = c.convert(path)
    duration = result.duration
    expected_samples = max(2, int(round(duration * 10.0)) + 1)
    assert abs(len(result.positions) - expected_samples) <= 2


# ---------------------------------------------------------------------------
# start_time offset
# ---------------------------------------------------------------------------


def test_timestamps_start_at_start_time() -> None:
    """start_time=5.0 → timestamps[0] == 5.0."""
    result = TrajectoryConverter(config=_SCARA_PLANNING).convert(
        [_START.copy(), _GOAL.copy()], start_time=5.0
    )
    assert math.isclose(result.timestamps[0], 5.0, rel_tol=1e-9)


def test_timestamps_end_at_start_time_plus_duration() -> None:
    """timestamps[-1] == start_time + duration."""
    start_time = 3.0
    result = TrajectoryConverter(config=_SCARA_PLANNING).convert(
        [_START.copy(), _GOAL.copy()], start_time=start_time
    )
    assert math.isclose(
        result.timestamps[-1], start_time + result.duration, rel_tol=1e-9
    )


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


def test_path_too_short_raises() -> None:
    """Path with 1 waypoint raises ValueError."""
    with pytest.raises(ValueError):
        TrajectoryConverter(config=_SCARA_PLANNING).convert([_START.copy()])


def test_empty_path_raises() -> None:
    """Empty path raises ValueError."""
    with pytest.raises(ValueError):
        TrajectoryConverter(config=_SCARA_PLANNING).convert([])
