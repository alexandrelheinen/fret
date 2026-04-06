"""Unit tests for fret.validation.metrics (Issue 09).

Validates:
- path_length: empty path, single waypoint, straight line, multi-segment.
- path_smoothness: too-short path, straight path, right-angle turn,
  U-turn, zero-length segment handling.
- min_obstacle_clearance: empty path, single waypoint, multi-waypoint,
  constrained minimum.
- tracking_rmse: empty input raises, mismatched lengths raise, zero error,
  known nonzero error, single DOF, multi-DOF.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Make the fret package importable without a full ROS build.
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.validation.metrics import (
    min_obstacle_clearance,
    path_length,
    path_smoothness,
    tracking_rmse,
)

# ---------------------------------------------------------------------------
# path_length
# ---------------------------------------------------------------------------


class TestPathLength(unittest.TestCase):
    """Tests for path_length()."""

    def test_empty_path_returns_zero(self) -> None:
        """An empty path has zero arc length."""
        self.assertEqual(path_length([]), 0.0)

    def test_single_waypoint_returns_zero(self) -> None:
        """A single-waypoint path has zero arc length."""
        self.assertEqual(path_length([[0.0, 0.0]]), 0.0)

    def test_two_identical_waypoints(self) -> None:
        """Two identical waypoints give zero length."""
        self.assertAlmostEqual(path_length([[1.0, 2.0], [1.0, 2.0]]), 0.0)

    def test_unit_step_1d(self) -> None:
        """One unit step in 1-D gives length 1.0."""
        self.assertAlmostEqual(path_length([[0.0], [1.0]]), 1.0)

    def test_unit_step_2d(self) -> None:
        """Diagonal step in 2-D gives sqrt(2)."""
        self.assertAlmostEqual(
            path_length([[0.0, 0.0], [1.0, 1.0]]),
            math.sqrt(2),
        )

    def test_three_waypoints_straight(self) -> None:
        """Three collinear waypoints: total length is sum of two segments."""
        path = [[0.0, 0.0], [1.0, 0.0], [3.0, 0.0]]
        self.assertAlmostEqual(path_length(path), 3.0)

    def test_right_angle_path(self) -> None:
        """Right-angle path: two unit segments give total length 2.0."""
        path = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
        self.assertAlmostEqual(path_length(path), 2.0)

    def test_four_dof_path(self) -> None:
        """4-DOF path: arc length matches manual Euclidean sum."""
        a = [0.0, 0.0, 0.0, 0.0]
        b = [1.0, 0.0, 0.0, 0.0]
        c = [1.0, 1.0, 0.0, 0.0]
        d = [1.0, 1.0, 0.1, 0.0]
        path = [a, b, c, d]
        expected = 1.0 + 1.0 + 0.1
        self.assertAlmostEqual(path_length(path), expected, places=10)

    def test_inconsistent_dof_raises(self) -> None:
        """Waypoints with differing DOF must raise ValueError."""
        with self.assertRaises(ValueError):
            path_length([[0.0, 0.0], [1.0, 0.0, 0.0]])


# ---------------------------------------------------------------------------
# path_smoothness
# ---------------------------------------------------------------------------


class TestPathSmoothness(unittest.TestCase):
    """Tests for path_smoothness()."""

    def test_empty_path(self) -> None:
        """Empty path has zero smoothness cost."""
        self.assertEqual(path_smoothness([]), 0.0)

    def test_single_waypoint(self) -> None:
        """Single waypoint has zero smoothness cost."""
        self.assertEqual(path_smoothness([[0.0, 0.0]]), 0.0)

    def test_two_waypoints(self) -> None:
        """Two waypoints have zero smoothness cost (no triple)."""
        self.assertEqual(path_smoothness([[0.0], [1.0]]), 0.0)

    def test_straight_line(self) -> None:
        """A perfectly straight path returns 0.0."""
        path = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]
        self.assertAlmostEqual(path_smoothness(path), 0.0)

    def test_right_angle_turn(self) -> None:
        """A 90-degree turn contributes π/2 radians."""
        path = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
        self.assertAlmostEqual(path_smoothness(path), math.pi / 2, places=10)

    def test_u_turn(self) -> None:
        """A U-turn (180 degrees) contributes π radians."""
        path = [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]]
        self.assertAlmostEqual(path_smoothness(path), math.pi, places=10)

    def test_two_right_angle_turns(self) -> None:
        """Two 90-degree turns accumulate to π radians total."""
        path = [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ]
        self.assertAlmostEqual(path_smoothness(path), math.pi, places=10)

    def test_zero_length_segment_skipped(self) -> None:
        """Duplicate waypoints (zero-length segment) contribute 0."""
        path = [[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]]
        # Both triples involving the duplicate should not contribute.
        self.assertAlmostEqual(path_smoothness(path), 0.0, places=10)

    def test_inconsistent_dof_raises(self) -> None:
        """Waypoints with differing DOF must raise ValueError."""
        with self.assertRaises(ValueError):
            path_smoothness([[0.0], [1.0], [2.0, 0.0]])


# ---------------------------------------------------------------------------
# min_obstacle_clearance
# ---------------------------------------------------------------------------


def _const_clearance(value: float):
    """Return a clearance function that always returns *value*."""
    return lambda wp: value


def _lookup_clearance(mapping: dict):
    """Return a clearance function that looks up each waypoint as a tuple."""
    return lambda wp: mapping[tuple(wp)]


class TestMinObstacleClearance(unittest.TestCase):
    """Tests for min_obstacle_clearance()."""

    def test_empty_path_returns_inf(self) -> None:
        """Empty path returns math.inf."""
        result = min_obstacle_clearance([], _const_clearance(1.0))
        self.assertEqual(result, math.inf)

    def test_single_waypoint(self) -> None:
        """Single waypoint returns its own clearance."""
        result = min_obstacle_clearance([[0.0]], _const_clearance(0.5))
        self.assertAlmostEqual(result, 0.5)

    def test_uniform_clearance(self) -> None:
        """Uniform clearance across all waypoints is returned directly."""
        path = [[0.0], [1.0], [2.0]]
        result = min_obstacle_clearance(path, _const_clearance(0.3))
        self.assertAlmostEqual(result, 0.3)

    def test_minimum_is_selected(self) -> None:
        """The minimum among varying clearances is returned."""
        mapping = {(0.0,): 0.5, (1.0,): 0.1, (2.0,): 0.4}
        path = [[0.0], [1.0], [2.0]]
        result = min_obstacle_clearance(path, _lookup_clearance(mapping))
        self.assertAlmostEqual(result, 0.1)

    def test_zero_clearance(self) -> None:
        """A waypoint on the obstacle surface returns 0.0."""
        path = [[0.0], [1.0]]
        result = min_obstacle_clearance(path, _const_clearance(0.0))
        self.assertAlmostEqual(result, 0.0)

    def test_clearance_uses_each_waypoint(self) -> None:
        """Verifies the function calls clearance_fn for every waypoint."""
        calls: list = []

        def counting_fn(wp: list) -> float:
            calls.append(wp)
            return 1.0

        path = [[0.0], [1.0], [2.0]]
        min_obstacle_clearance(path, counting_fn)
        self.assertEqual(len(calls), 3)


# ---------------------------------------------------------------------------
# tracking_rmse
# ---------------------------------------------------------------------------


class TestTrackingRmse(unittest.TestCase):
    """Tests for tracking_rmse()."""

    def test_empty_sequences_raise(self) -> None:
        """Empty sequences must raise ValueError."""
        with self.assertRaises(ValueError):
            tracking_rmse([], [])

    def test_mismatched_lengths_raise(self) -> None:
        """Sequences of different lengths must raise ValueError."""
        with self.assertRaises(ValueError):
            tracking_rmse([[0.0]], [[0.0], [1.0]])

    def test_zero_dof_raises(self) -> None:
        """Zero-DOF waypoints must raise ValueError."""
        with self.assertRaises(ValueError):
            tracking_rmse([[]], [[]])

    def test_inconsistent_dof_raises(self) -> None:
        """Mixed-DOF waypoints must raise ValueError."""
        with self.assertRaises(ValueError):
            tracking_rmse([[0.0, 0.0]], [[0.0]])

    def test_perfect_tracking(self) -> None:
        """Zero error between identical sequences returns 0.0."""
        actual = [[0.0, 1.0], [0.5, 0.5], [1.0, 0.0]]
        desired = [[0.0, 1.0], [0.5, 0.5], [1.0, 0.0]]
        self.assertAlmostEqual(tracking_rmse(actual, desired), 0.0)

    def test_single_step_1d(self) -> None:
        """Single step 1-D: RMSE equals |actual - desired|."""
        actual = [[2.0]]
        desired = [[1.0]]
        # error = 1.0, sqrt(1.0^2 / (1*1)) = 1.0
        self.assertAlmostEqual(tracking_rmse(actual, desired), 1.0)

    def test_single_step_2d(self) -> None:
        """Single step 2-D: RMSE = sqrt(sum_sq / (1*2))."""
        actual = [[1.0, 0.0]]
        desired = [[0.0, 0.0]]
        # sum_sq = 1.0, N=1, D=2 → sqrt(1/2)
        self.assertAlmostEqual(tracking_rmse(actual, desired), math.sqrt(0.5))

    def test_multi_step_known_value(self) -> None:
        """Multi-step known example: manual verification."""
        # 2 steps, 1 DOF: errors = [1.0, 3.0]
        # RMSE = sqrt((1^2 + 3^2) / (2*1)) = sqrt(10/2) = sqrt(5)
        actual = [[1.0], [3.0]]
        desired = [[0.0], [0.0]]
        self.assertAlmostEqual(tracking_rmse(actual, desired), math.sqrt(5.0))

    def test_symmetry(self) -> None:
        """RMSE is symmetric: swapping actual and desired gives same result."""
        actual = [[0.1, 0.2], [0.3, 0.4]]
        desired = [[0.5, 0.6], [0.7, 0.8]]
        self.assertAlmostEqual(
            tracking_rmse(actual, desired),
            tracking_rmse(desired, actual),
        )

    def test_single_joint_multi_step(self) -> None:
        """Regression: 4-step 1-DOF with constant error 0.1."""
        # RMSE = sqrt(4 * 0.01 / 4) = sqrt(0.01) = 0.1
        actual = [[0.1], [0.1], [0.1], [0.1]]
        desired = [[0.0], [0.0], [0.0], [0.0]]
        self.assertAlmostEqual(tracking_rmse(actual, desired), 0.1)


# ---------------------------------------------------------------------------
# Package import test
# ---------------------------------------------------------------------------


class TestValidationPackageImport(unittest.TestCase):
    """Smoke tests for the fret.validation package import."""

    def test_public_symbols_importable(self) -> None:
        """All public symbols must be importable from fret.validation."""
        from fret.validation import (  # noqa: F401
            GateResult,
            QualityGate,
            ScenarioReport,
            evaluate_gates,
            format_report,
            min_obstacle_clearance,
            path_length,
            path_smoothness,
            tracking_rmse,
        )

    def test_metrics_module_importable(self) -> None:
        """fret.validation.metrics must be importable directly."""
        import fret.validation.metrics  # noqa: F401

    def test_quality_gates_module_importable(self) -> None:
        """fret.validation.quality_gates must be importable directly."""
        import fret.validation.quality_gates  # noqa: F401


if __name__ == "__main__":
    unittest.main()
