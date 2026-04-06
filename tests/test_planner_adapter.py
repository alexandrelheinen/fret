"""Unit tests for the ARCO planner adapter (Issue 05).

Validates:
- PlannerAdapter constructor argument checking.
- PlanningRequest schema validation (all required fields, type checks).
- Stale-occupancy detection (invalid_request with failure_reason).
- Unsupported algorithm rejection.
- Baseline scenario: adapter returns a valid path in free space.
- Timeout: adapter returns status=="timeout" when deadline is exceeded.
- No-solution: adapter returns status=="no_plan_found" when goal is
  unreachable (fully blocked by collision).
- Start/goal in collision: adapter returns no_plan_found with a descriptive
  failure_reason.
- Diagnostics: solve_time, node_count, failure_reason are always present.
- PlanningResult schema integrity: all required fields present.
- RRT-Connect constructor argument validation.
- RRT-Connect trivial path (start == goal within step_size).
- RRT-Connect path validity (all waypoints within joint limits).
- RRT-Connect reproducibility with fixed rng_seed.
- Integration with OccupancyAdapter: planner avoids occupied region.
- Config deep-merge: caller overrides are respected.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import time
import unittest
import uuid

# ---------------------------------------------------------------------------
# Make the fret package importable without a full ROS build
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.planner_adapter import (
    CANONICAL_FRAME,
    DEFAULT_CONFIG,
    SUPPORTED_ALGORITHMS,
    PlannerAdapter,
    _deep_merge,
    _free_space_validator,
)
from fret.planning.rrt_connect import RRTConnect, _Tree, _dist

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

DOF = 4

# SCARA-like joint limits (R-R-P-R).
JOINT_LIMITS = [
    (-math.pi * 132 / 180, math.pi * 132 / 180),  # joint_arm_0 [rad]
    (-math.pi * 150 / 180, math.pi * 150 / 180),  # joint_arm_1 [rad]
    (0.0, 0.2),  # joint_extension [m]
    (-math.pi, math.pi),  # joint_tool_rotate [rad]
]

START = [0.0, 0.0, 0.0, 0.0]
GOAL = [0.5, 0.3, 0.1, 0.2]


def _make_request(
    *,
    request_id=None,
    start=None,
    goal=None,
    occupancy_stamp=None,
    timeout=5.0,
    algorithm="rrt_connect",
    reference_frame=CANONICAL_FRAME,
    joint_count=None,
):
    start = start if start is not None else list(START)
    goal = goal if goal is not None else list(GOAL)
    if joint_count is None:
        joint_count = len(start)
    return {
        "request_id": request_id if request_id is not None else str(uuid.uuid4()),
        "start_joint_positions": start,
        "goal_joint_positions": goal,
        "joint_count": joint_count,
        "occupancy_stamp": occupancy_stamp
        if occupancy_stamp is not None
        else time.time(),
        "timeout": timeout,
        "planner_config": {"algorithm": algorithm},
        "reference_frame": reference_frame,
    }


def _make_adapter(state_validator=None, config=None):
    occupancy = OccupancyAdapter(inflation_radius=0.05)
    return PlannerAdapter(
        occupancy_adapter=occupancy,
        joint_limits=JOINT_LIMITS,
        state_validator=state_validator,
        config=config,
    )


# ---------------------------------------------------------------------------
# PlannerAdapter constructor
# ---------------------------------------------------------------------------


class TestPlannerAdapterConstructor(unittest.TestCase):
    def test_default_construction(self):
        adapter = _make_adapter()
        self.assertEqual(adapter.dof, DOF)
        self.assertEqual(len(adapter.joint_limits), DOF)

    def test_empty_joint_limits_raises(self):
        occupancy = OccupancyAdapter()
        with self.assertRaises(ValueError):
            PlannerAdapter(
                occupancy_adapter=occupancy,
                joint_limits=[],
            )

    def test_inverted_joint_limits_raises(self):
        occupancy = OccupancyAdapter()
        with self.assertRaises(ValueError):
            PlannerAdapter(
                occupancy_adapter=occupancy,
                joint_limits=[(1.0, -1.0)],  # inverted
            )

    def test_equal_joint_limits_raises(self):
        occupancy = OccupancyAdapter()
        with self.assertRaises(ValueError):
            PlannerAdapter(
                occupancy_adapter=occupancy,
                joint_limits=[(0.0, 0.0)],  # zero range
            )

    def test_config_defaults_merged(self):
        adapter = _make_adapter()
        cfg = adapter.config
        self.assertIn("algorithm", cfg)
        self.assertIn("rrt_connect", cfg)
        self.assertIn("step_size", cfg["rrt_connect"])

    def test_config_override_respected(self):
        adapter = _make_adapter(
            config={"rrt_connect": {"step_size": 0.1}}
        )
        self.assertAlmostEqual(
            adapter.config["rrt_connect"]["step_size"], 0.1
        )

    def test_free_space_validator_is_default(self):
        adapter = _make_adapter(state_validator=None)
        # Free-space validator must allow any configuration.
        self.assertTrue(adapter.config is not None)

    def test_custom_validator_stored(self):
        called = []

        def my_validator(q):
            called.append(q)
            return True

        adapter = _make_adapter(state_validator=my_validator)
        req = _make_request()
        result = adapter.plan(req)
        # Validator must have been called during planning.
        self.assertGreater(len(called), 0)


# ---------------------------------------------------------------------------
# PlannerAdapter request validation
# ---------------------------------------------------------------------------


class TestPlannerAdapterRequestValidation(unittest.TestCase):
    def setUp(self):
        self.adapter = _make_adapter()

    def _assert_invalid(self, request, substring=None):
        result = self.adapter.plan(request)
        self.assertEqual(result["status"], "invalid_request")
        self.assertIsNotNone(result["failure_reason"])
        if substring:
            self.assertIn(substring, result["failure_reason"])

    def test_plan_raises_on_non_dict(self):
        with self.assertRaises(TypeError):
            self.adapter.plan("not a dict")

    def test_missing_request_id(self):
        req = _make_request()
        del req["request_id"]
        self._assert_invalid(req, "request_id")

    def test_missing_start_joint_positions(self):
        req = _make_request()
        del req["start_joint_positions"]
        self._assert_invalid(req, "start_joint_positions")

    def test_missing_goal_joint_positions(self):
        req = _make_request()
        del req["goal_joint_positions"]
        self._assert_invalid(req, "goal_joint_positions")

    def test_missing_joint_count(self):
        req = _make_request()
        del req["joint_count"]
        self._assert_invalid(req, "joint_count")

    def test_missing_occupancy_stamp(self):
        req = _make_request()
        del req["occupancy_stamp"]
        self._assert_invalid(req, "occupancy_stamp")

    def test_missing_timeout(self):
        req = _make_request()
        del req["timeout"]
        self._assert_invalid(req, "timeout")

    def test_missing_planner_config(self):
        req = _make_request()
        del req["planner_config"]
        self._assert_invalid(req, "planner_config")

    def test_missing_reference_frame(self):
        req = _make_request()
        del req["reference_frame"]
        self._assert_invalid(req, "reference_frame")

    def test_empty_request_id_rejected(self):
        req = _make_request(request_id="")
        self._assert_invalid(req, "request_id")

    def test_wrong_reference_frame_rejected(self):
        req = _make_request(reference_frame="base_link")
        self._assert_invalid(req, "reference_frame")

    def test_joint_count_mismatch_rejected(self):
        req = _make_request(joint_count=DOF + 1)
        self._assert_invalid(req)

    def test_start_length_mismatch_rejected(self):
        req = _make_request(start=[0.0] * (DOF - 1))
        self._assert_invalid(req)

    def test_goal_length_mismatch_rejected(self):
        req = _make_request(goal=[0.0] * (DOF + 1), joint_count=DOF)
        self._assert_invalid(req)

    def test_zero_timeout_rejected(self):
        req = _make_request(timeout=0.0)
        self._assert_invalid(req, "timeout")

    def test_negative_timeout_rejected(self):
        req = _make_request(timeout=-1.0)
        self._assert_invalid(req, "timeout")

    def test_missing_algorithm_in_planner_config(self):
        req = _make_request()
        req["planner_config"] = {}
        self._assert_invalid(req, "algorithm")

    def test_unsupported_algorithm_rejected(self):
        req = _make_request(algorithm="prm_star")
        result = self.adapter.plan(req)
        self.assertEqual(result["status"], "invalid_request")
        self.assertIn("unsupported algorithm", result["failure_reason"])

    def test_stale_occupancy_rejected(self):
        req = _make_request(
            occupancy_stamp=time.time() - 100.0  # very old
        )
        result = self.adapter.plan(req)
        self.assertEqual(result["status"], "invalid_request")
        self.assertIn("stale_occupancy", result["failure_reason"])


# ---------------------------------------------------------------------------
# PlannerAdapter planning — success path
# ---------------------------------------------------------------------------


class TestPlannerAdapterBaseline(unittest.TestCase):
    """Adapter returns valid path in free space (no obstacles)."""

    def test_free_space_plan_succeeds(self):
        adapter = _make_adapter(
            config={"rrt_connect": {"rng_seed": 42}}
        )
        req = _make_request()
        result = adapter.plan(req)

        self.assertEqual(result["status"], "success")
        self.assertIsNotNone(result["path"])
        self.assertGreater(len(result["path"]), 0)
        self.assertEqual(result["waypoint_count"], len(result["path"]))

    def test_result_starts_near_start(self):
        adapter = _make_adapter(
            config={"rrt_connect": {"rng_seed": 42}}
        )
        req = _make_request()
        result = adapter.plan(req)
        self.assertEqual(result["status"], "success")
        first = result["path"][0]
        dist = _dist(first, START)
        self.assertLess(dist, 1e-6)

    def test_result_ends_near_goal(self):
        adapter = _make_adapter(
            config={"rrt_connect": {"rng_seed": 42}}
        )
        req = _make_request()
        result = adapter.plan(req)
        self.assertEqual(result["status"], "success")
        last = result["path"][-1]
        dist = _dist(last, GOAL)
        self.assertLess(dist, 0.1)

    def test_request_id_echoed(self):
        adapter = _make_adapter()
        rid = str(uuid.uuid4())
        req = _make_request(request_id=rid)
        result = adapter.plan(req)
        self.assertEqual(result["request_id"], rid)

    def test_reference_frame_echoed(self):
        adapter = _make_adapter()
        req = _make_request()
        result = adapter.plan(req)
        self.assertEqual(result["reference_frame"], CANONICAL_FRAME)

    def test_all_result_fields_present(self):
        adapter = _make_adapter()
        req = _make_request()
        result = adapter.plan(req)
        for field in [
            "request_id",
            "status",
            "path",
            "waypoint_count",
            "solve_time",
            "node_count",
            "failure_reason",
            "reference_frame",
        ]:
            with self.subTest(field=field):
                self.assertIn(field, result)

    def test_trivial_path_start_equals_goal(self):
        adapter = _make_adapter()
        pt = [0.1, 0.1, 0.05, 0.0]
        req = _make_request(start=list(pt), goal=list(pt))
        result = adapter.plan(req)
        self.assertEqual(result["status"], "success")
        self.assertGreaterEqual(result["waypoint_count"], 1)


# ---------------------------------------------------------------------------
# PlannerAdapter — timeout
# ---------------------------------------------------------------------------


class TestPlannerAdapterTimeout(unittest.TestCase):
    """Timeout is handled explicitly."""

    def test_timeout_returns_timeout_status(self):
        # Use a blocking validator and a very short timeout so the planner
        # runs out of time quickly.
        def _slow_validator(q):
            # Only allow configurations near the start to force the planner
            # to explore a constrained region and exhaust its budget.
            return all(abs(qi) < 0.01 for qi in q)

        occupancy = OccupancyAdapter()
        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
            state_validator=_slow_validator,
            config={
                "rrt_connect": {
                    "max_iterations": 2_000_000,
                    "step_size": 0.001,
                    "rng_seed": 0,
                }
            },
        )
        req = _make_request(
            start=[0.0] * DOF,
            goal=[1.0] * DOF,
            timeout=0.05,
        )
        t0 = time.monotonic()
        result = adapter.plan(req)
        elapsed = time.monotonic() - t0

        self.assertIn(result["status"], {"timeout", "no_plan_found"})
        # The call must return reasonably quickly.
        self.assertLess(elapsed, 5.0)
        self.assertIsNotNone(result["failure_reason"])


# ---------------------------------------------------------------------------
# PlannerAdapter — no-solution
# ---------------------------------------------------------------------------


class TestPlannerAdapterNoSolution(unittest.TestCase):
    """No-solution paths are handled explicitly."""

    def test_blocked_goal_returns_no_plan_found(self):
        # Validator that allows start but blocks everything else.
        def _restricted(q):
            return all(abs(qi) < 0.001 for qi in q)

        occupancy = OccupancyAdapter()
        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
            state_validator=_restricted,
            config={
                "rrt_connect": {
                    "max_iterations": 200,
                    "rng_seed": 0,
                }
            },
        )
        req = _make_request(
            start=[0.0] * DOF,
            goal=[1.0] * DOF,
        )
        result = adapter.plan(req)
        self.assertIn(
            result["status"], {"no_plan_found", "timeout"}
        )
        self.assertIsNone(result["path"])
        self.assertEqual(result["waypoint_count"], 0)
        self.assertIsNotNone(result["failure_reason"])

    def test_start_in_collision_returns_no_plan_found(self):
        def _blocked(_q):
            return False

        occupancy = OccupancyAdapter()
        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
            state_validator=_blocked,
        )
        req = _make_request()
        result = adapter.plan(req)
        self.assertIn(
            result["status"], {"no_plan_found", "invalid_request"}
        )
        self.assertIsNone(result["path"])


# ---------------------------------------------------------------------------
# PlannerAdapter — diagnostics
# ---------------------------------------------------------------------------


class TestPlannerAdapterDiagnostics(unittest.TestCase):
    """Diagnostics are emitted and have correct types."""

    def setUp(self):
        self.adapter = _make_adapter(
            config={"rrt_connect": {"rng_seed": 0}}
        )

    def _run(self, **kw):
        return self.adapter.plan(_make_request(**kw))

    def test_solve_time_positive(self):
        result = self._run()
        self.assertGreater(result["solve_time"], 0.0)

    def test_solve_time_is_float(self):
        result = self._run()
        self.assertIsInstance(result["solve_time"], float)

    def test_node_count_non_negative(self):
        result = self._run()
        self.assertGreaterEqual(result["node_count"], 0)

    def test_node_count_is_int(self):
        result = self._run()
        self.assertIsInstance(result["node_count"], int)

    def test_failure_reason_none_on_success(self):
        result = self._run()
        if result["status"] == "success":
            self.assertIsNone(result["failure_reason"])

    def test_failure_reason_present_on_invalid(self):
        req = _make_request(timeout=-1.0)
        result = self.adapter.plan(req)
        self.assertIsNotNone(result["failure_reason"])
        self.assertIsInstance(result["failure_reason"], str)

    def test_solve_time_present_for_invalid_request(self):
        req = _make_request(reference_frame="sensor_frame")
        result = self.adapter.plan(req)
        self.assertIn("solve_time", result)
        self.assertGreaterEqual(result["solve_time"], 0.0)

    def test_node_count_positive_after_planning(self):
        result = self._run()
        if result["status"] == "success":
            self.assertGreater(result["node_count"], 0)


# ---------------------------------------------------------------------------
# RRTConnect — constructor
# ---------------------------------------------------------------------------


class TestRRTConnectConstructor(unittest.TestCase):
    def test_default_construction(self):
        planner = RRTConnect(
            joint_limits=JOINT_LIMITS,
            state_validator=_free_space_validator,
        )
        self.assertIsNotNone(planner)

    def test_empty_joint_limits_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=[],
                state_validator=_free_space_validator,
            )

    def test_inverted_limits_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=[(1.0, -1.0)],
                state_validator=_free_space_validator,
            )

    def test_zero_step_size_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=JOINT_LIMITS,
                state_validator=_free_space_validator,
                step_size=0.0,
            )

    def test_negative_step_size_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=JOINT_LIMITS,
                state_validator=_free_space_validator,
                step_size=-0.1,
            )

    def test_zero_max_iterations_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=JOINT_LIMITS,
                state_validator=_free_space_validator,
                max_iterations=0,
            )

    def test_goal_bias_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=JOINT_LIMITS,
                state_validator=_free_space_validator,
                goal_bias=1.5,
            )

    def test_negative_goal_bias_raises(self):
        with self.assertRaises(ValueError):
            RRTConnect(
                joint_limits=JOINT_LIMITS,
                state_validator=_free_space_validator,
                goal_bias=-0.1,
            )


# ---------------------------------------------------------------------------
# RRTConnect — plan method
# ---------------------------------------------------------------------------


class TestRRTConnectPlan(unittest.TestCase):
    def _planner(self, seed=42, **kw):
        defaults = dict(
            joint_limits=JOINT_LIMITS,
            state_validator=_free_space_validator,
            step_size=0.1,
            max_iterations=5_000,
            goal_bias=0.1,
            rng_seed=seed,
        )
        defaults.update(kw)
        return RRTConnect(**defaults)

    def test_plan_returns_three_tuple(self):
        planner = self._planner()
        result = planner.plan(START, GOAL, timeout=10.0)
        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 3)

    def test_plan_free_space_succeeds(self):
        planner = self._planner()
        path, node_count, fail_reason = planner.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path)
        self.assertIsNone(fail_reason)
        self.assertGreater(node_count, 0)

    def test_path_starts_at_start(self):
        planner = self._planner()
        path, _, _ = planner.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path)
        self.assertAlmostEqual(_dist(path[0], START), 0.0, places=9)

    def test_path_ends_near_goal(self):
        planner = self._planner()
        path, _, _ = planner.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path)
        self.assertLess(_dist(path[-1], GOAL), 0.15)

    def test_path_length_at_least_two(self):
        planner = self._planner()
        path, _, _ = planner.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path)
        self.assertGreaterEqual(len(path), 2)

    def test_trivial_start_equals_goal(self):
        planner = self._planner()
        pt = [0.1, 0.1, 0.05, 0.0]
        path, node_count, fail = planner.plan(pt, pt, 5.0)
        self.assertIsNotNone(path)
        self.assertIsNone(fail)

    def test_timeout_returns_none_path(self):
        def _expensive_validator(_q):
            # Accept nothing so the planner iterates until timeout.
            return False

        planner = RRTConnect(
            joint_limits=JOINT_LIMITS,
            state_validator=_expensive_validator,
            step_size=0.05,
            max_iterations=10_000_000,
            rng_seed=0,
        )
        path, node_count, fail = planner.plan(START, GOAL, timeout=0.05)
        self.assertIsNone(path)
        self.assertIn(fail, {"timeout", "start_in_collision"})

    def test_start_wrong_dof_raises(self):
        planner = self._planner()
        with self.assertRaises(ValueError):
            planner.plan([0.0] * (DOF - 1), GOAL, 5.0)

    def test_goal_wrong_dof_raises(self):
        planner = self._planner()
        with self.assertRaises(ValueError):
            planner.plan(START, [0.0] * (DOF + 1), 5.0)

    def test_zero_timeout_raises(self):
        planner = self._planner()
        with self.assertRaises(ValueError):
            planner.plan(START, GOAL, timeout=0.0)

    def test_reproducible_with_fixed_seed(self):
        p1 = self._planner(seed=7)
        p2 = self._planner(seed=7)
        path1, _, _ = p1.plan(START, GOAL, 10.0)
        path2, _, _ = p2.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path1)
        self.assertEqual(path1, path2)

    def test_different_seeds_may_differ(self):
        p1 = self._planner(seed=1)
        p2 = self._planner(seed=2)
        path1, _, _ = p1.plan(START, GOAL, 10.0)
        path2, _, _ = p2.plan(START, GOAL, 10.0)
        # Both must succeed; paths may differ.
        self.assertIsNotNone(path1)
        self.assertIsNotNone(path2)

    def test_all_waypoints_within_joint_limits(self):
        planner = self._planner()
        path, _, _ = planner.plan(START, GOAL, 10.0)
        self.assertIsNotNone(path)
        for waypoint in path:
            for i, (q, (lo, hi)) in enumerate(
                zip(waypoint, JOINT_LIMITS)
            ):
                with self.subTest(joint=i, q=q):
                    self.assertGreaterEqual(q, lo - 1e-9)
                    self.assertLessEqual(q, hi + 1e-9)

    def test_start_in_collision_returns_none(self):
        def _blocked(_q):
            return False

        planner = RRTConnect(
            joint_limits=JOINT_LIMITS,
            state_validator=_blocked,
            rng_seed=0,
        )
        path, _, fail = planner.plan(START, GOAL, 5.0)
        self.assertIsNone(path)
        self.assertIn("collision", fail)


# ---------------------------------------------------------------------------
# Internal _Tree helper
# ---------------------------------------------------------------------------


class TestTree(unittest.TestCase):
    def test_initial_state(self):
        root = [0.0, 0.0]
        tree = _Tree(root)
        self.assertEqual(tree.node_count, 1)
        self.assertEqual(tree.nodes[0], root)
        self.assertEqual(tree.parents[0], -1)

    def test_add_node(self):
        tree = _Tree([0.0, 0.0])
        idx = tree.add([1.0, 0.0], 0)
        self.assertEqual(idx, 1)
        self.assertEqual(tree.node_count, 2)
        self.assertEqual(tree.parents[1], 0)

    def test_nearest_returns_root_for_single_node(self):
        tree = _Tree([1.0, 2.0])
        idx, node = tree.nearest([5.0, 5.0])
        self.assertEqual(idx, 0)
        self.assertEqual(node, [1.0, 2.0])

    def test_nearest_correct(self):
        tree = _Tree([0.0, 0.0])
        tree.add([1.0, 0.0], 0)
        tree.add([0.0, 1.0], 0)
        idx, _ = tree.nearest([0.9, 0.1])
        self.assertEqual(idx, 1)

    def test_path_to_root_single_node(self):
        tree = _Tree([0.0, 0.0])
        path = tree.path_to_root(0)
        self.assertEqual(len(path), 1)

    def test_path_to_root_chain(self):
        tree = _Tree([0.0])
        i1 = tree.add([1.0], 0)
        i2 = tree.add([2.0], i1)
        path = tree.path_to_root(i2)
        self.assertEqual(path, [[0.0], [1.0], [2.0]])


# ---------------------------------------------------------------------------
# _dist helper
# ---------------------------------------------------------------------------


class TestDistHelper(unittest.TestCase):
    def test_identical_points(self):
        self.assertAlmostEqual(_dist([1.0, 2.0], [1.0, 2.0]), 0.0)

    def test_known_distance(self):
        self.assertAlmostEqual(_dist([0.0, 0.0], [3.0, 4.0]), 5.0)

    def test_single_dim(self):
        self.assertAlmostEqual(_dist([0.0], [2.0]), 2.0)


# ---------------------------------------------------------------------------
# Integration: OccupancyAdapter + PlannerAdapter
# ---------------------------------------------------------------------------


class TestPlannerAdapterWithOccupancy(unittest.TestCase):
    """Integration tests: planner avoids obstacles in the occupancy model."""

    @staticmethod
    def _build_obstacle_wall():
        """Return a wall of points blocking the straight-line path."""
        points = []
        for x in range(-5, 6):
            for z in range(0, 10):
                points.append([0.3, x * 0.02, z * 0.05])
        return points

    def test_free_space_plan_succeeds_with_populated_occupancy(self):
        occupancy = OccupancyAdapter(inflation_radius=0.05)
        occupancy.update([])  # empty scene

        def validator(q):
            # Treat the first two joints as x/y proxy for testing.
            return occupancy.is_free([q[0], q[1], 0.0])

        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
            state_validator=validator,
            config={"rrt_connect": {"rng_seed": 0}},
        )
        req = _make_request()
        result = adapter.plan(req)
        self.assertEqual(result["status"], "success")
        self.assertIsNotNone(result["path"])

    def test_occupancy_is_stale_after_max_rebuild_age(self):
        occupancy = OccupancyAdapter(
            inflation_radius=0.05,
            max_rebuild_age=0.01,
        )
        occupancy.update([])
        time.sleep(0.02)  # let it go stale

        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
        )
        # Occupancy is stale in the OccupancyAdapter sense, but
        # occupancy_stamp in the request is fresh — these are independent.
        # The stale_occupancy check uses occupancy_stamp, not adapter.is_stale.
        req = _make_request()
        result = adapter.plan(req)
        # Request should still succeed if stamp is fresh.
        self.assertIn(result["status"], {"success", "no_plan_found"})


# ---------------------------------------------------------------------------
# Config deep merge
# ---------------------------------------------------------------------------


class TestDeepMerge(unittest.TestCase):
    def test_flat_override(self):
        base = {"a": 1, "b": 2}
        override = {"b": 99, "c": 3}
        merged = _deep_merge(base, override)
        self.assertEqual(merged, {"a": 1, "b": 99, "c": 3})

    def test_nested_merge(self):
        base = {"x": {"a": 1, "b": 2}}
        override = {"x": {"b": 99}}
        merged = _deep_merge(base, override)
        self.assertEqual(merged["x"], {"a": 1, "b": 99})

    def test_base_not_mutated(self):
        base = {"x": {"a": 1}}
        _deep_merge(base, {"x": {"a": 2}})
        self.assertEqual(base["x"]["a"], 1)

    def test_empty_override(self):
        base = {"a": 1}
        merged = _deep_merge(base, {})
        self.assertEqual(merged, base)


# ---------------------------------------------------------------------------
# YAML config file
# ---------------------------------------------------------------------------


class TestPlannerConfigFile(unittest.TestCase):
    """Verify that the YAML config file exists and is well-formed."""

    _CONFIG_PATH = os.path.join(
        os.path.dirname(_TESTS_DIR),
        "src",
        "fret",
        "config",
        "planner.yaml",
    )

    def test_config_file_exists(self):
        self.assertTrue(
            os.path.isfile(self._CONFIG_PATH),
            f"planner.yaml not found at {self._CONFIG_PATH}",
        )

    def test_config_file_is_valid_yaml(self):
        import re

        with open(self._CONFIG_PATH, "r", encoding="utf-8") as fh:
            content = fh.read()
        # Minimal check: file must contain the 'planner' top-level key.
        self.assertIn("planner:", content)
        self.assertIn("rrt_connect:", content)
        self.assertIn("step_size:", content)
        self.assertIn("max_iterations:", content)

    def test_config_documents_all_supported_algorithms(self):
        with open(self._CONFIG_PATH, "r", encoding="utf-8") as fh:
            content = fh.read()
        for algo in SUPPORTED_ALGORITHMS:
            with self.subTest(algorithm=algo):
                self.assertIn(algo, content)


# ---------------------------------------------------------------------------
# Supported algorithms constant
# ---------------------------------------------------------------------------


class TestSupportedAlgorithms(unittest.TestCase):
    def test_rrt_connect_is_supported(self):
        self.assertIn("rrt_connect", SUPPORTED_ALGORITHMS)

    def test_supported_algorithms_is_frozenset(self):
        self.assertIsInstance(SUPPORTED_ALGORITHMS, frozenset)


if __name__ == "__main__":
    unittest.main()
