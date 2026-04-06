"""Unit tests for the ARCO-FRET trajectory converter (Issue 06).

Validates:
- TrajectoryConverter constructor argument checking.
- _deep_merge, _expand_per_joint, _trapezoid_segment_time helpers.
- TrajectoryResult schema completeness.
- Single-waypoint (trivial) and multi-waypoint paths.
- Time parameterization correctness (trapezoidal profile times).
- Per-joint velocity and acceleration limit enforcement.
- Synchronization across joints (all segments use the maximum joint time).
- Joint-limit checking returns "infeasible" status.
- Malformed-input checking returns "invalid_input" status.
- Empty-path edge case.
- Determinism: identical input produces identical output.
- request_id propagation from PlanningResult (handover contract).
- TrajectoryConverter config deep-merge.
- trajectory.yaml config file exists and contains required keys.
- VALID_STATUSES and CANONICAL_FRAME public constants.

All tests are deterministic and run in-process; no ROS runtime required.
"""

from __future__ import annotations

import math
import os
import sys
import unittest
import uuid

# ---------------------------------------------------------------------------
# Ensure src/ is on the path when running outside of an installed package.
# ---------------------------------------------------------------------------
_SRC = os.path.join(os.path.dirname(__file__), "..", "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from fret.planning.trajectory_converter import (
    CANONICAL_FRAME,
    DEFAULT_CONFIG,
    VALID_STATUSES,
    TrajectoryConverter,
    _deep_merge,
    _expand_per_joint,
    _trapezoid_segment_time,
)

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

DOF = 4
JOINT_LIMITS = [  # SCARA-like (R-R-P-R)
    (-math.pi * 132 / 180, math.pi * 132 / 180),
    (-math.pi * 150 / 180, math.pi * 150 / 180),
    (0.0, 0.2),
    (-math.pi, math.pi),
]
V_MAX = 1.0  # rad/s or m/s (broadcast)
A_MAX = 2.0  # rad/s² or m/s²

START = [0.0, 0.0, 0.0, 0.0]
GOAL = [0.5, 0.3, 0.1, 0.2]

# ---------------------------------------------------------------------------
# Tolerance for floating-point comparisons
# ---------------------------------------------------------------------------
_RTOL = 1e-9


def _make_converter(config=None) -> TrajectoryConverter:
    cfg = {"max_velocity": V_MAX, "max_acceleration": A_MAX}
    if config:
        cfg.update(config)
    return TrajectoryConverter(JOINT_LIMITS, config=cfg)


# ===========================================================================
# Helper: _trapezoid_segment_time
# ===========================================================================


class TestTrapezoidSegmentTime(unittest.TestCase):
    """Unit tests for the internal trapezoidal time helper."""

    def test_zero_distance_returns_zero(self):
        self.assertEqual(_trapezoid_segment_time(0.0, 1.0, 2.0), 0.0)

    def test_negative_distance_returns_zero(self):
        self.assertEqual(_trapezoid_segment_time(-1.0, 1.0, 2.0), 0.0)

    def test_triangular_profile(self):
        # distance=0.5, v_max=2.0, a_max=2.0
        # v_peak = sqrt(0.5 * 2.0) = 1.0 <= 2.0 → triangular
        # t = 2 * 1.0 / 2.0 = 1.0 s
        t = _trapezoid_segment_time(0.5, 2.0, 2.0)
        self.assertAlmostEqual(t, 1.0, places=12)

    def test_trapezoidal_profile(self):
        # distance=2.0, v_max=1.0, a_max=1.0
        # v_peak = sqrt(2.0) ≈ 1.414 > 1.0 → trapezoidal
        # t_ramp = 1.0, d_ramp = 0.5, d_coast = 1.0, t_coast = 1.0
        # t_total = 2 + 1 = 3.0 s
        t = _trapezoid_segment_time(2.0, 1.0, 1.0)
        self.assertAlmostEqual(t, 3.0, places=12)

    def test_borderline_triangular_equals_trapezoidal(self):
        # distance=1.0, v_max=1.0, a_max=1.0
        # v_peak = sqrt(1.0) = 1.0 = v_max → triangular (borderline)
        # t = 2 * 1.0 / 1.0 = 2.0 s
        t = _trapezoid_segment_time(1.0, 1.0, 1.0)
        self.assertAlmostEqual(t, 2.0, places=12)

    def test_small_distance(self):
        # distance=0.001, v_max=1.0, a_max=2.0
        # v_peak = sqrt(0.001 * 2.0) ≈ 0.0447 <= 1.0 → triangular
        # t = 2 * 0.0447 / 2.0 ≈ 0.04472 s
        t = _trapezoid_segment_time(0.001, 1.0, 2.0)
        expected = 2.0 * math.sqrt(0.001 / 2.0)
        self.assertAlmostEqual(t, expected, places=12)

    def test_large_distance_trapezoidal(self):
        # distance=10.0, v_max=1.0, a_max=2.0
        # v_peak = sqrt(10 * 2) ≈ 4.47 > 1.0 → trapezoidal
        # t_ramp = 0.5, d_ramp = 0.25, d_coast = 9.5, t_coast = 9.5
        # t_total = 2*0.5 + 9.5 = 10.5
        t = _trapezoid_segment_time(10.0, 1.0, 2.0)
        t_ramp = 1.0 / 2.0
        d_ramp = 0.5 * 2.0 * t_ramp**2
        d_coast = 10.0 - 2 * d_ramp
        expected = 2 * t_ramp + d_coast / 1.0
        self.assertAlmostEqual(t, expected, places=12)

    def test_time_covers_distance(self):
        """Verify trapezoidal time actually covers the required distance."""
        for dist, v_max, a_max in [
            (1.0, 1.0, 2.0),
            (0.5, 0.5, 1.0),
            (3.0, 2.0, 4.0),
        ]:
            with self.subTest(dist=dist, v_max=v_max, a_max=a_max):
                t = _trapezoid_segment_time(dist, v_max, a_max)
                t_ramp = min(v_max / a_max, t / 2.0)
                d_covered = 2 * 0.5 * a_max * t_ramp**2 + (
                    t - 2 * t_ramp
                ) * min(a_max * t_ramp, v_max)
                self.assertAlmostEqual(d_covered, dist, places=9)

    def test_result_is_positive_for_positive_distance(self):
        t = _trapezoid_segment_time(0.1, 1.0, 2.0)
        self.assertGreater(t, 0.0)


# ===========================================================================
# Helper: _expand_per_joint
# ===========================================================================


class TestExpandPerJoint(unittest.TestCase):
    """Unit tests for the per-joint value expansion helper."""

    def test_scalar_broadcast(self):
        result = _expand_per_joint(1.5, 4, "max_velocity")
        self.assertEqual(result, [1.5, 1.5, 1.5, 1.5])

    def test_list_passthrough(self):
        values = [1.0, 0.5, 0.25, 2.0]
        result = _expand_per_joint(values, 4, "max_velocity")
        self.assertEqual(result, values)

    def test_integer_scalar_broadcast(self):
        result = _expand_per_joint(2, 3, "max_acceleration")
        self.assertEqual(result, [2.0, 2.0, 2.0])

    def test_wrong_list_length_raises(self):
        with self.assertRaises(ValueError):
            _expand_per_joint([1.0, 2.0], 4, "max_velocity")

    def test_non_positive_scalar_raises(self):
        with self.assertRaises(ValueError):
            _expand_per_joint(0.0, 4, "max_velocity")
        with self.assertRaises(ValueError):
            _expand_per_joint(-1.0, 4, "max_velocity")

    def test_non_positive_list_element_raises(self):
        with self.assertRaises(ValueError):
            _expand_per_joint([1.0, 0.0, 1.0, 1.0], 4, "max_velocity")

    def test_invalid_type_raises(self):
        with self.assertRaises(ValueError):
            _expand_per_joint("fast", 4, "max_velocity")

    def test_returns_float_list(self):
        result = _expand_per_joint(2, 3, "max_velocity")
        for v in result:
            self.assertIsInstance(v, float)


# ===========================================================================
# Helper: _deep_merge
# ===========================================================================


class TestDeepMerge(unittest.TestCase):
    """Unit tests for the config deep-merge helper."""

    def test_flat_override(self):
        result = _deep_merge({"a": 1, "b": 2}, {"b": 99})
        self.assertEqual(result, {"a": 1, "b": 99})

    def test_nested_merge(self):
        base = {"outer": {"a": 1, "b": 2}}
        override = {"outer": {"b": 99}}
        result = _deep_merge(base, override)
        self.assertEqual(result["outer"], {"a": 1, "b": 99})

    def test_base_not_mutated(self):
        base = {"a": 1}
        _deep_merge(base, {"a": 2})
        self.assertEqual(base["a"], 1)

    def test_empty_override_returns_copy_of_base(self):
        base = {"a": 1, "b": {"c": 3}}
        result = _deep_merge(base, {})
        self.assertEqual(result, base)
        self.assertIsNot(result, base)

    def test_new_key_in_override(self):
        result = _deep_merge({"a": 1}, {"z": 99})
        self.assertEqual(result["z"], 99)
        self.assertEqual(result["a"], 1)


# ===========================================================================
# Constructor validation
# ===========================================================================


class TestTrajectoryConverterConstructor(unittest.TestCase):
    """Tests for TrajectoryConverter.__init__ argument checking."""

    def test_valid_construction_succeeds(self):
        tc = TrajectoryConverter(JOINT_LIMITS)
        self.assertIsInstance(tc, TrajectoryConverter)

    def test_empty_joint_limits_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter([])

    def test_inverted_joint_limit_raises(self):
        bad = list(JOINT_LIMITS)
        bad[0] = (1.0, -1.0)  # lower > upper
        with self.assertRaises(ValueError):
            TrajectoryConverter(bad)

    def test_equal_joint_limit_raises(self):
        bad = list(JOINT_LIMITS)
        bad[1] = (0.5, 0.5)  # lower == upper
        with self.assertRaises(ValueError):
            TrajectoryConverter(bad)

    def test_single_joint_is_valid(self):
        tc = TrajectoryConverter([(-1.0, 1.0)])
        self.assertIsNotNone(tc)

    def test_config_override_max_velocity(self):
        tc = TrajectoryConverter(JOINT_LIMITS, config={"max_velocity": 0.5})
        self.assertEqual(tc._v_max, [0.5] * DOF)

    def test_config_override_max_acceleration(self):
        tc = TrajectoryConverter(
            JOINT_LIMITS, config={"max_acceleration": 4.0}
        )
        self.assertEqual(tc._a_max, [4.0] * DOF)

    def test_config_per_joint_velocity(self):
        v = [0.5, 0.6, 0.3, 0.8]
        tc = TrajectoryConverter(JOINT_LIMITS, config={"max_velocity": v})
        self.assertEqual(tc._v_max, v)

    def test_config_per_joint_velocity_wrong_length_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter(
                JOINT_LIMITS, config={"max_velocity": [1.0, 2.0]}
            )

    def test_non_positive_velocity_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter(JOINT_LIMITS, config={"max_velocity": 0.0})

    def test_non_positive_acceleration_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter(
                JOINT_LIMITS, config={"max_acceleration": -1.0}
            )

    def test_non_positive_min_segment_time_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter(JOINT_LIMITS, config={"min_segment_time": 0.0})

    def test_non_positive_command_rate_raises(self):
        with self.assertRaises(ValueError):
            TrajectoryConverter(JOINT_LIMITS, config={"command_rate_hz": 0.0})

    def test_default_config_fallback(self):
        tc = TrajectoryConverter(JOINT_LIMITS)
        self.assertEqual(tc._v_max, [DEFAULT_CONFIG["max_velocity"]] * DOF)
        self.assertEqual(tc._a_max, [DEFAULT_CONFIG["max_acceleration"]] * DOF)


# ===========================================================================
# TrajectoryResult schema
# ===========================================================================


class TestTrajectoryResultSchema(unittest.TestCase):
    """Tests for the completeness and types of TrajectoryResult dicts."""

    REQUIRED_KEYS = {
        "request_id",
        "status",
        "waypoints",
        "timestamps",
        "velocities",
        "accelerations",
        "total_time",
        "waypoint_count",
        "failure_reason",
        "reference_frame",
        "command_rate_hz",
    }

    def setUp(self):
        self.tc = _make_converter()

    def _assert_schema(self, result: dict, path_len: int):
        for key in self.REQUIRED_KEYS:
            self.assertIn(key, result, f"missing key: {key}")
        self.assertIsInstance(result["request_id"], str)
        self.assertIn(result["status"], VALID_STATUSES)
        self.assertIsInstance(result["waypoints"], list)
        self.assertIsInstance(result["timestamps"], list)
        self.assertIsInstance(result["velocities"], list)
        self.assertIsInstance(result["accelerations"], list)
        self.assertIsInstance(result["total_time"], float)
        self.assertIsInstance(result["waypoint_count"], int)
        self.assertIsInstance(result["failure_reason"], str)
        self.assertEqual(result["reference_frame"], CANONICAL_FRAME)
        self.assertIsInstance(result["command_rate_hz"], float)
        if result["status"] == "success":
            self.assertEqual(len(result["waypoints"]), path_len)
            self.assertEqual(len(result["timestamps"]), path_len)
            self.assertEqual(len(result["velocities"]), path_len)
            self.assertEqual(len(result["accelerations"]), path_len)
            self.assertEqual(result["waypoint_count"], path_len)
            self.assertEqual(result["failure_reason"], "")
        else:
            self.assertNotEqual(result["failure_reason"], "")

    def test_success_schema_two_waypoints(self):
        result = self.tc.convert([START, GOAL])
        self._assert_schema(result, 2)

    def test_success_schema_single_waypoint(self):
        result = self.tc.convert([START])
        self._assert_schema(result, 1)

    def test_invalid_input_schema(self):
        result = self.tc.convert([])
        self._assert_schema(result, 0)

    def test_infeasible_schema(self):
        # Waypoint outside joint limits.
        bad_wp = [99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([START, bad_wp])
        self._assert_schema(result, 0)

    def test_velocities_shape(self):
        result = self.tc.convert([START, GOAL])
        for vel in result["velocities"]:
            self.assertEqual(len(vel), DOF)

    def test_accelerations_shape(self):
        result = self.tc.convert([START, GOAL])
        for acc in result["accelerations"]:
            self.assertEqual(len(acc), DOF)

    def test_waypoints_are_copies(self):
        path = [list(START), list(GOAL)]
        result = self.tc.convert(path)
        # Mutating original path must not affect result.
        path[0][0] = 999.0
        self.assertEqual(result["waypoints"][0][0], START[0])


# ===========================================================================
# Time parameterization
# ===========================================================================


class TestTimeParameterization(unittest.TestCase):
    """Tests for timestamp correctness and kinematic limit enforcement."""

    def setUp(self):
        self.tc = _make_converter()

    # --- Single-waypoint (trivial) -----------------------------------------

    def test_single_waypoint_total_time_zero(self):
        result = self.tc.convert([START])
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["total_time"], 0.0)
        self.assertEqual(result["timestamps"], [0.0])

    # --- Two waypoints -------------------------------------------------------

    def test_two_waypoints_first_timestamp_zero(self):
        result = self.tc.convert([START, GOAL])
        self.assertEqual(result["timestamps"][0], 0.0)

    def test_two_waypoints_total_time_positive(self):
        result = self.tc.convert([START, GOAL])
        self.assertGreater(result["total_time"], 0.0)

    def test_two_waypoints_timestamps_monotone(self):
        result = self.tc.convert([START, GOAL])
        ts = result["timestamps"]
        for i in range(len(ts) - 1):
            self.assertLess(ts[i], ts[i + 1])

    def test_two_waypoints_total_time_equals_last_timestamp(self):
        result = self.tc.convert([START, GOAL])
        self.assertAlmostEqual(
            result["total_time"], result["timestamps"][-1], places=12
        )

    # --- Multi-waypoint ------------------------------------------------------

    def test_multi_waypoint_timestamps_count(self):
        path = [START, [0.2, 0.1, 0.05, 0.1], GOAL]
        result = self.tc.convert(path)
        self.assertEqual(len(result["timestamps"]), 3)

    def test_multi_waypoint_timestamps_monotone(self):
        path = [START, [0.2, 0.1, 0.05, 0.1], GOAL]
        result = self.tc.convert(path)
        ts = result["timestamps"]
        for i in range(len(ts) - 1):
            self.assertLess(ts[i], ts[i + 1])

    # --- Trapezoidal profile correctness -------------------------------------

    def test_single_joint_triangular_time(self):
        """One active joint: triangular profile (v_peak <= v_max)."""
        tc = TrajectoryConverter(
            [(-2.0, 2.0)],
            config={"max_velocity": 2.0, "max_acceleration": 2.0},
        )
        # distance=0.5, v_peak = sqrt(0.5*2)=1.0 <= 2.0 → triangular
        # t = 2*1.0/2.0 = 1.0 s
        result = tc.convert([[0.0], [0.5]])
        self.assertAlmostEqual(result["total_time"], 1.0, places=12)

    def test_single_joint_trapezoidal_time(self):
        """One active joint: trapezoidal profile (v_peak > v_max)."""
        tc = TrajectoryConverter(
            [(-5.0, 5.0)],
            config={"max_velocity": 1.0, "max_acceleration": 1.0},
        )
        # distance=2.0, t = 3.0 s (computed manually above)
        result = tc.convert([[0.0], [2.0]])
        self.assertAlmostEqual(result["total_time"], 3.0, places=12)

    def test_segment_time_is_max_over_joints(self):
        """Synchronization: segment time equals the slowest joint's time."""
        # Joint 0: distance=1.0 → t=2.0 s (triangular, borderline)
        # Joint 1: distance=0.1 → t=2*sqrt(0.1/2)/1 ≈ 0.447 s (triangular)
        # Segment time must be max = 2.0 s.
        tc = TrajectoryConverter(
            [(-5.0, 5.0), (-5.0, 5.0)],
            config={"max_velocity": 1.0, "max_acceleration": 1.0},
        )
        result = tc.convert([[0.0, 0.0], [1.0, 0.1]])
        t_j0 = _trapezoid_segment_time(1.0, 1.0, 1.0)  # 2.0
        t_j1 = _trapezoid_segment_time(0.1, 1.0, 1.0)
        expected = max(t_j0, t_j1)
        self.assertAlmostEqual(result["total_time"], expected, places=12)

    def test_zero_displacement_uses_min_segment_time(self):
        """Identical consecutive waypoints clamp to min_segment_time."""
        min_t = 0.001
        tc = TrajectoryConverter(
            JOINT_LIMITS,
            config={
                "max_velocity": 1.0,
                "max_acceleration": 2.0,
                "min_segment_time": min_t,
            },
        )
        result = tc.convert([START, START])
        self.assertAlmostEqual(result["total_time"], min_t, places=12)

    def test_large_displacement_respects_velocity_limit(self):
        """Large move: trapezoidal profile must respect v_max."""
        v_max = 0.5
        a_max = 1.0
        dist = 5.0
        tc = TrajectoryConverter(
            [(-10.0, 10.0)],
            config={"max_velocity": v_max, "max_acceleration": a_max},
        )
        result = tc.convert([[0.0], [dist]])
        expected = _trapezoid_segment_time(dist, v_max, a_max)
        self.assertAlmostEqual(result["total_time"], expected, places=12)

    def test_timestamps_start_at_zero(self):
        path = [START, GOAL]
        result = self.tc.convert(path)
        self.assertEqual(result["timestamps"][0], 0.0)

    def test_total_time_is_sum_of_segment_times(self):
        """Total time equals sum of per-segment trapezoidal times."""
        mid = [0.2, 0.1, 0.05, 0.1]
        path = [START, mid, GOAL]
        result = self.tc.convert(path)

        def seg_time(q_a, q_b):
            t = 0.0
            for j in range(DOF):
                t_j = _trapezoid_segment_time(
                    abs(q_b[j] - q_a[j]), V_MAX, A_MAX
                )
                t = max(t, t_j)
            return max(t, DEFAULT_CONFIG["min_segment_time"])

        expected = seg_time(START, mid) + seg_time(mid, GOAL)
        self.assertAlmostEqual(result["total_time"], expected, places=9)


# ===========================================================================
# Velocity and acceleration fields
# ===========================================================================


class TestVelocityAndAccelerationFields(unittest.TestCase):
    """Tests for velocity/acceleration values in the TrajectoryResult."""

    def setUp(self):
        self.tc = _make_converter()

    def test_velocities_all_zero_two_waypoints(self):
        result = self.tc.convert([START, GOAL])
        for vel in result["velocities"]:
            for v in vel:
                self.assertEqual(v, 0.0)

    def test_accelerations_all_zero_two_waypoints(self):
        result = self.tc.convert([START, GOAL])
        for acc in result["accelerations"]:
            for a in acc:
                self.assertEqual(a, 0.0)

    def test_velocities_all_zero_single_waypoint(self):
        result = self.tc.convert([START])
        for vel in result["velocities"]:
            for v in vel:
                self.assertEqual(v, 0.0)

    def test_velocities_shape_multi_waypoint(self):
        path = [START, [0.1, 0.1, 0.05, 0.0], GOAL]
        result = self.tc.convert(path)
        self.assertEqual(len(result["velocities"]), 3)
        for vel in result["velocities"]:
            self.assertEqual(len(vel), DOF)

    def test_accelerations_shape_multi_waypoint(self):
        path = [START, [0.1, 0.1, 0.05, 0.0], GOAL]
        result = self.tc.convert(path)
        self.assertEqual(len(result["accelerations"]), 3)
        for acc in result["accelerations"]:
            self.assertEqual(len(acc), DOF)


# ===========================================================================
# Failure handling
# ===========================================================================


class TestFailureHandling(unittest.TestCase):
    """Tests for infeasible and invalid-input failure modes."""

    def setUp(self):
        self.tc = _make_converter()

    # --- invalid_input -------------------------------------------------------

    def test_empty_path_is_invalid_input(self):
        result = self.tc.convert([])
        self.assertEqual(result["status"], "invalid_input")
        self.assertNotEqual(result["failure_reason"], "")

    def test_wrong_dof_waypoint_is_invalid_input(self):
        result = self.tc.convert([[0.0, 0.0]])  # 2 values, DOF=4
        self.assertEqual(result["status"], "invalid_input")

    def test_wrong_dof_mid_path_is_invalid_input(self):
        bad = [START, [0.1, 0.2, 0.3], GOAL]  # middle has 3 values
        result = self.tc.convert(bad)
        self.assertEqual(result["status"], "invalid_input")

    def test_failure_reason_nonempty_on_invalid_input(self):
        result = self.tc.convert([])
        self.assertTrue(len(result["failure_reason"]) > 0)

    # --- infeasible ----------------------------------------------------------

    def test_waypoint_above_upper_limit_is_infeasible(self):
        bad_wp = [99.0, 0.0, 0.0, 0.0]  # joint 0 >> upper limit
        result = self.tc.convert([START, bad_wp])
        self.assertEqual(result["status"], "infeasible")

    def test_waypoint_below_lower_limit_is_infeasible(self):
        bad_wp = [-99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([START, bad_wp])
        self.assertEqual(result["status"], "infeasible")

    def test_start_outside_limits_is_infeasible(self):
        bad_start = [99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([bad_start, GOAL])
        self.assertEqual(result["status"], "infeasible")

    def test_infeasible_failure_reason_mentions_joint(self):
        bad_wp = [99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([START, bad_wp])
        # failure_reason should mention 'joint' or a joint index
        self.assertIn("joint", result["failure_reason"].lower())

    def test_infeasible_has_empty_waypoints(self):
        bad_wp = [99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([START, bad_wp])
        self.assertEqual(result["waypoints"], [])

    def test_infeasible_has_empty_timestamps(self):
        bad_wp = [99.0, 0.0, 0.0, 0.0]
        result = self.tc.convert([START, bad_wp])
        self.assertEqual(result["timestamps"], [])

    # --- Boundary values are accepted ----------------------------------------

    def test_waypoint_at_lower_limit_is_valid(self):
        lo_wp = [lo for lo, _ in JOINT_LIMITS]
        result = self.tc.convert([lo_wp, START])
        self.assertEqual(result["status"], "success")

    def test_waypoint_at_upper_limit_is_valid(self):
        hi_wp = [hi for _, hi in JOINT_LIMITS]
        result = self.tc.convert([START, hi_wp])
        self.assertEqual(result["status"], "success")


# ===========================================================================
# Determinism
# ===========================================================================


class TestDeterminism(unittest.TestCase):
    """Identical inputs must produce identical outputs."""

    def test_same_path_same_output(self):
        tc = _make_converter()
        r1 = tc.convert([START, GOAL], request_id="abc-123")
        r2 = tc.convert([START, GOAL], request_id="abc-123")
        self.assertEqual(r1["timestamps"], r2["timestamps"])
        self.assertEqual(r1["total_time"], r2["total_time"])
        self.assertEqual(r1["status"], r2["status"])

    def test_different_paths_different_times(self):
        tc = _make_converter()
        r1 = tc.convert([START, GOAL])
        r2 = tc.convert([START, [0.1, 0.1, 0.05, 0.1]])
        self.assertNotEqual(r1["total_time"], r2["total_time"])

    def test_symmetric_path_same_total_time(self):
        """Forward and reverse paths take the same total time."""
        tc = _make_converter()
        forward = tc.convert([START, GOAL])
        backward = tc.convert([GOAL, START])
        self.assertAlmostEqual(
            forward["total_time"], backward["total_time"], places=9
        )


# ===========================================================================
# Handover contract: request_id propagation
# ===========================================================================


class TestHandoverContract(unittest.TestCase):
    """Tests for the planner-to-controller handover contract."""

    def setUp(self):
        self.tc = _make_converter()

    def test_request_id_propagated_when_provided(self):
        rid = str(uuid.uuid4())
        result = self.tc.convert([START, GOAL], request_id=rid)
        self.assertEqual(result["request_id"], rid)

    def test_request_id_auto_generated_when_not_provided(self):
        result = self.tc.convert([START, GOAL])
        # Must be a non-empty string.
        self.assertIsInstance(result["request_id"], str)
        self.assertTrue(len(result["request_id"]) > 0)

    def test_auto_generated_ids_are_unique(self):
        r1 = self.tc.convert([START, GOAL])
        r2 = self.tc.convert([START, GOAL])
        self.assertNotEqual(r1["request_id"], r2["request_id"])

    def test_reference_frame_is_world(self):
        result = self.tc.convert([START, GOAL])
        self.assertEqual(result["reference_frame"], "world")

    def test_command_rate_hz_matches_config(self):
        rate = 200.0
        tc = TrajectoryConverter(
            JOINT_LIMITS, config={"command_rate_hz": rate}
        )
        result = tc.convert([START, GOAL])
        self.assertEqual(result["command_rate_hz"], rate)

    def test_failure_result_contains_request_id(self):
        rid = "fixed-id"
        result = self.tc.convert([], request_id=rid)
        self.assertEqual(result["request_id"], rid)


# ===========================================================================
# Public constants
# ===========================================================================


class TestPublicConstants(unittest.TestCase):
    """Tests for CANONICAL_FRAME and VALID_STATUSES."""

    def test_canonical_frame_is_world(self):
        self.assertEqual(CANONICAL_FRAME, "world")

    def test_valid_statuses_is_frozenset(self):
        self.assertIsInstance(VALID_STATUSES, frozenset)

    def test_valid_statuses_contains_success(self):
        self.assertIn("success", VALID_STATUSES)

    def test_valid_statuses_contains_infeasible(self):
        self.assertIn("infeasible", VALID_STATUSES)

    def test_valid_statuses_contains_invalid_input(self):
        self.assertIn("invalid_input", VALID_STATUSES)

    def test_default_config_has_required_keys(self):
        for key in (
            "max_velocity",
            "max_acceleration",
            "min_segment_time",
            "command_rate_hz",
        ):
            self.assertIn(key, DEFAULT_CONFIG)


# ===========================================================================
# Config file
# ===========================================================================


class TestTrajectoryConfigFile(unittest.TestCase):
    """Tests for trajectory.yaml existence and validity."""

    _CONFIG_PATH = os.path.join(
        os.path.dirname(__file__),
        "..",
        "src",
        "fret",
        "config",
        "trajectory.yaml",
    )

    def test_config_file_exists(self):
        self.assertTrue(
            os.path.isfile(self._CONFIG_PATH),
            f"trajectory.yaml not found at {self._CONFIG_PATH}",
        )

    def test_config_file_is_valid_yaml(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("pyyaml not available")
        with open(self._CONFIG_PATH) as fh:
            data = yaml.safe_load(fh)
        self.assertIn("trajectory", data)
        traj = data["trajectory"]
        for key in (
            "max_velocity",
            "max_acceleration",
            "min_segment_time",
            "command_rate_hz",
        ):
            self.assertIn(key, traj, f"'{key}' missing from trajectory.yaml")

    def test_config_file_values_are_positive(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("pyyaml not available")
        with open(self._CONFIG_PATH) as fh:
            data = yaml.safe_load(fh)
        traj = data["trajectory"]
        for key in (
            "max_velocity",
            "max_acceleration",
            "min_segment_time",
            "command_rate_hz",
        ):
            val = traj.get(key)
            if isinstance(val, (int, float)):
                self.assertGreater(
                    val, 0, f"trajectory.yaml '{key}' must be positive"
                )


# ===========================================================================
# Package import
# ===========================================================================


class TestPackageImport(unittest.TestCase):
    """TrajectoryConverter must be importable from fret.planning."""

    def test_import_from_planning_package(self):
        from fret.planning import TrajectoryConverter as TC

        self.assertIs(TC, TrajectoryConverter)

    def test_in_all(self):
        import fret.planning as pkg

        self.assertIn("TrajectoryConverter", pkg.__all__)


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    unittest.main()
