"""Unit tests for fret.validation.metrics."""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from fret.validation.metrics import (
    min_obstacle_clearance,
    path_length,
    path_smoothness,
    tracking_rmse,
)

# ---------------------------------------------------------------------------
# path_length
# ---------------------------------------------------------------------------


def test_path_length_empty() -> None:
    assert path_length([]) == 0.0


def test_path_length_single() -> None:
    assert path_length([np.array([1.0, 2.0, 3.0])]) == 0.0


def test_path_length_two_equal() -> None:
    q = np.array([0.5, 0.5])
    assert path_length([q, q]) == pytest.approx(0.0)


def test_path_length_two_different() -> None:
    a = np.array([0.0, 0.0])
    b = np.array([3.0, 4.0])
    assert path_length([a, b]) == pytest.approx(5.0)


def test_path_length_three_collinear() -> None:
    a = np.array([0.0])
    b = np.array([1.0])
    c = np.array([3.0])
    assert path_length([a, b, c]) == pytest.approx(3.0)


def test_path_length_unit_vectors() -> None:
    # Six unit steps of length 1 each → total = 6.0
    path = [np.array([float(i)]) for i in range(7)]
    assert path_length(path) == pytest.approx(6.0)


# ---------------------------------------------------------------------------
# path_smoothness
# ---------------------------------------------------------------------------


def test_smoothness_straight() -> None:
    # Collinear points: all segments point in the same direction → 0.0
    path = [np.array([float(i), 0.0]) for i in range(5)]
    assert path_smoothness(path) == pytest.approx(0.0, abs=1e-10)


def test_smoothness_right_angle() -> None:
    # Three points forming a 90-degree turn
    p = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0])]
    assert path_smoothness(p) == pytest.approx(math.pi / 2)


def test_smoothness_u_turn() -> None:
    # Three collinear points that reverse direction → π
    p = [np.array([0.0]), np.array([1.0]), np.array([0.0])]
    assert path_smoothness(p) == pytest.approx(math.pi)


def test_smoothness_empty() -> None:
    assert path_smoothness([]) == 0.0


def test_smoothness_two_points() -> None:
    p = [np.array([0.0, 0.0]), np.array([1.0, 1.0])]
    assert path_smoothness(p) == 0.0


def test_smoothness_zigzag() -> None:
    # Alternating left/right turns → smoothness > 0
    p = [
        np.array([0.0, 0.0]),
        np.array([1.0, 1.0]),
        np.array([2.0, 0.0]),
        np.array([3.0, 1.0]),
    ]
    assert path_smoothness(p) > 0.0


# ---------------------------------------------------------------------------
# min_obstacle_clearance
# ---------------------------------------------------------------------------


def _make_kinematics(translation: np.ndarray) -> MagicMock:
    """Return a mock kinematics object whose FK always returns *translation*."""
    T = np.eye(4)
    T[:3, 3] = translation
    kin = MagicMock()
    kin.forward_kinematics.return_value = T
    return kin


def test_clearance_empty_path() -> None:
    occ = MagicMock()
    kin = MagicMock()
    assert min_obstacle_clearance([], occ, kin) == math.inf


def test_clearance_free_space() -> None:
    occ = MagicMock()
    occ.clearance.return_value = 0.5
    kin = _make_kinematics(np.array([1.0, 0.0, 0.0]))
    path = [np.array([0.0, 0.0, 0.0])]
    result = min_obstacle_clearance(path, occ, kin)
    assert result == pytest.approx(0.5)
    assert result > 0.0


def test_clearance_in_collision() -> None:
    occ = MagicMock()
    occ.clearance.return_value = -0.05
    kin = _make_kinematics(np.array([0.0, 0.0, 0.0]))
    path = [np.array([0.0])]
    result = min_obstacle_clearance(path, occ, kin)
    assert result == pytest.approx(-0.05)
    assert result < 0.0


def test_clearance_minimum_selected() -> None:
    # Three waypoints with clearances 0.3, 0.1, 0.5 → min is 0.1
    clearances = [0.3, 0.1, 0.5]
    occ = MagicMock()
    occ.clearance.side_effect = clearances
    kin = _make_kinematics(np.zeros(3))
    path = [np.array([float(i)]) for i in range(3)]
    result = min_obstacle_clearance(path, occ, kin)
    assert result == pytest.approx(0.1)


# ---------------------------------------------------------------------------
# tracking_rmse
# ---------------------------------------------------------------------------


def test_rmse_identical() -> None:
    traj = [np.array([1.0, 2.0, 3.0])] * 4
    assert tracking_rmse(traj, traj) == pytest.approx(0.0)


def test_rmse_constant_offset() -> None:
    # Reference [0, 0], actual [1, 0] for all steps → RMSE = sqrt(0.5)
    ref = [np.array([0.0, 0.0])] * 3
    act = [np.array([1.0, 0.0])] * 3
    expected = math.sqrt(0.5)
    assert tracking_rmse(ref, act) == pytest.approx(expected)


def test_rmse_length_mismatch_raises() -> None:
    ref = [np.array([0.0])] * 3
    act = [np.array([0.0])] * 2
    with pytest.raises(ValueError, match="same length"):
        tracking_rmse(ref, act)


def test_rmse_shape_mismatch_raises() -> None:
    ref = [np.array([0.0, 0.0])]
    act = [np.array([0.0, 0.0, 0.0])]
    with pytest.raises(ValueError, match="same shape"):
        tracking_rmse(ref, act)


def test_rmse_single_step() -> None:
    # Single step: ref=[0,0], actual=[3,4] → RMSE = sqrt((9+16)/2) = sqrt(12.5)
    ref = [np.array([0.0, 0.0])]
    act = [np.array([3.0, 4.0])]
    expected = math.sqrt((9.0 + 16.0) / 2.0)
    assert tracking_rmse(ref, act) == pytest.approx(expected)
