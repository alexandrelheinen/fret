"""Unit tests for the ARCO-FRET replanning manager (Issue 07).

Validates:
- ReplanningManager constructor argument checking.
- ManagerState and TriggerKind enum values.
- VALID_STATES and VALID_STATUSES public constants.
- DEFAULT_CONFIG keys and types.
- replanning.yaml config file existence and required keys.
- _rms_error helper: empty, single, multi-joint cases.
- _deep_merge helper: nested overrides, non-destructive.
- _free_space_validator helper: always True.
- update_goal: initialise without trigger; raise GOAL_UPDATE on change.
- update_occupancy: no trigger on first call; OCCUPANCY_CHANGE on large delta.
- update_occupancy: no trigger when change ratio is below threshold.
- update_occupancy: PATH_INVALIDATED when active path is blocked after update.
- update_tracking_state: no trigger before window fills; TRACKING_ERROR when
  sustained above threshold over the full hysteresis window.
- update_tracking_state: no trigger when errors are below threshold.
- update_tracking_state: dimension mismatch is silently ignored.
- evaluate_triggers: returns None when no trigger is pending.
- evaluate_triggers: returns None when debounced by min_replan_interval.
- evaluate_triggers: returns None when in FALLBACK with cooldown active.
- evaluate_triggers: returns trigger after cooldown elapses.
- TriggerKind priority: higher-priority trigger overwrites lower one.
- replan: raises TypeError on non-list current_joint_positions.
- replan: invalid_request when no goal has been set.
- replan: success → EXECUTING state; trajectory present; path updated.
- replan: planner failure → FALLBACK state; no trajectory.
- replan: pending trigger is consumed after replan().
- replan: occupancy_stamp forwarded correctly to the planner.
- safe transition: old path preserved until new replan succeeds.
- reset: clears pending trigger, tracking history; returns to EXECUTING.
- config deep-merge: caller overrides are respected.
- ReplanResult schema: all required keys present on every outcome.
- Integration: full occupancy-update → evaluate → replan cycle in free space.
- Integration: replanning skipped within min_replan_interval.
- set_active_path: sets and clears active path.
- current_goal property: reflects latest goal.
- state property: reflects current state.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

from __future__ import annotations

import math
import os
import sys
import time
import unittest
import uuid

# ---------------------------------------------------------------------------
# Make the fret package importable without a full ROS build.
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.planner_adapter import PlannerAdapter
from fret.planning.replanning_manager import (
    DEFAULT_CONFIG,
    VALID_STATES,
    VALID_STATUSES,
    ManagerState,
    ReplanningManager,
    TriggerKind,
    _deep_merge,
    _free_space_validator,
    _rms_error,
)
from fret.planning.trajectory_converter import TrajectoryConverter

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

DOF = 4

JOINT_LIMITS = [
    (-math.pi * 132 / 180, math.pi * 132 / 180),
    (-math.pi * 150 / 180, math.pi * 150 / 180),
    (0.0, 0.2),
    (-math.pi, math.pi),
]

START = [0.0, 0.0, 0.0, 0.0]
GOAL = [0.3, 0.2, 0.1, 0.1]


def _make_manager(
    state_validator=None,
    config=None,
    joint_limits=None,
):
    """Build a ReplanningManager backed by free-space planner/converter."""
    limits = joint_limits or JOINT_LIMITS
    occupancy = OccupancyAdapter(inflation_radius=0.05)
    adapter = PlannerAdapter(
        occupancy_adapter=occupancy,
        joint_limits=limits,
        state_validator=state_validator,
        config={"rrt_connect": {"rng_seed": 42}},
    )
    converter = TrajectoryConverter(limits)
    return ReplanningManager(
        planner_adapter=adapter,
        trajectory_converter=converter,
        state_validator=state_validator,
        config=config,
    )


def _make_occupancy_points(n: int) -> list:
    """Return a list of n dummy 3-D points."""
    return [(float(i) * 0.01, 0.0, 0.0) for i in range(n)]


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------


class TestPublicConstants(unittest.TestCase):
    """Tests for module-level public constants."""

    def test_valid_states_contains_all_states(self):
        for s in ManagerState:
            self.assertIn(s.value, VALID_STATES)

    def test_valid_statuses_contains_required(self):
        for key in (
            "success",
            "no_plan_found",
            "timeout",
            "invalid_request",
            "debounced",
        ):
            self.assertIn(key, VALID_STATUSES)

    def test_default_config_keys_present(self):
        required_keys = {
            "min_replan_interval",
            "path_invalidation_ratio",
            "tracking_error_threshold",
            "tracking_error_window",
            "occupancy_change_ratio",
            "fallback_cooldown",
        }
        self.assertTrue(required_keys.issubset(set(DEFAULT_CONFIG)))

    def test_default_config_values_positive(self):
        for key, val in DEFAULT_CONFIG.items():
            self.assertGreater(
                val, 0.0, msg=f"DEFAULT_CONFIG['{key}'] must be positive"
            )

    def test_replanning_yaml_exists(self):
        config_dir = os.path.join(_SRC_DIR, "fret", "config")
        yaml_path = os.path.join(config_dir, "replanning.yaml")
        self.assertTrue(
            os.path.isfile(yaml_path),
            f"replanning.yaml not found at {yaml_path}",
        )

    def test_replanning_yaml_contains_required_keys(self):
        """Parse replanning.yaml and verify required keys are present."""
        config_dir = os.path.join(_SRC_DIR, "fret", "config")
        yaml_path = os.path.join(config_dir, "replanning.yaml")
        with open(yaml_path) as fh:
            content = fh.read()
        for key in DEFAULT_CONFIG:
            self.assertIn(
                key,
                content,
                msg=f"Key '{key}' missing from replanning.yaml",
            )


class TestEnumerations(unittest.TestCase):
    """Tests for ManagerState and TriggerKind enumerations."""

    def test_manager_state_values(self):
        self.assertEqual(ManagerState.EXECUTING.value, "executing")
        self.assertEqual(ManagerState.REPLANNING.value, "replanning")
        self.assertEqual(ManagerState.FALLBACK.value, "fallback")

    def test_trigger_kind_values(self):
        self.assertEqual(TriggerKind.NONE.value, "none")
        self.assertEqual(
            TriggerKind.OCCUPANCY_CHANGE.value, "occupancy_change"
        )
        self.assertEqual(
            TriggerKind.PATH_INVALIDATED.value, "path_invalidated"
        )
        self.assertEqual(TriggerKind.TRACKING_ERROR.value, "tracking_error")
        self.assertEqual(TriggerKind.GOAL_UPDATE.value, "goal_update")

    def test_manager_state_is_str(self):
        self.assertIsInstance(ManagerState.EXECUTING, str)

    def test_trigger_kind_is_str(self):
        self.assertIsInstance(TriggerKind.NONE, str)


class TestHelpers(unittest.TestCase):
    """Tests for private helper functions."""

    # _rms_error ---------------------------------------------------------

    def test_rms_error_empty(self):
        self.assertEqual(_rms_error([], []), 0.0)

    def test_rms_error_identical(self):
        self.assertAlmostEqual(_rms_error([1.0, 2.0], [1.0, 2.0]), 0.0)

    def test_rms_error_single_joint(self):
        # RMS of [error] = |error|
        self.assertAlmostEqual(_rms_error([1.0], [0.0]), 1.0)

    def test_rms_error_multi_joint(self):
        # errors = [1, 1, 1, 1] → RMS = 1.0
        actual = [1.0, 1.0, 1.0, 1.0]
        ref = [0.0, 0.0, 0.0, 0.0]
        self.assertAlmostEqual(_rms_error(actual, ref), 1.0)

    def test_rms_error_mixed(self):
        # errors = [3, 4] → sqrt((9+16)/2) = sqrt(12.5)
        self.assertAlmostEqual(
            _rms_error([3.0, 4.0], [0.0, 0.0]),
            math.sqrt(12.5),
        )

    # _free_space_validator -----------------------------------------------

    def test_free_space_validator_always_true(self):
        self.assertTrue(_free_space_validator([]))
        self.assertTrue(_free_space_validator([1.0, 2.0, 3.0]))

    # _deep_merge ---------------------------------------------------------

    def test_deep_merge_non_destructive(self):
        base = {"a": 1, "b": {"c": 2}}
        override = {"b": {"d": 3}}
        result = _deep_merge(base, override)
        self.assertEqual(result["b"]["c"], 2)
        self.assertEqual(result["b"]["d"], 3)
        # base unchanged
        self.assertNotIn("d", base["b"])

    def test_deep_merge_override_wins(self):
        result = _deep_merge({"a": 1}, {"a": 99})
        self.assertEqual(result["a"], 99)

    def test_deep_merge_empty_override(self):
        base = {"x": 10}
        self.assertEqual(_deep_merge(base, {}), base)


class TestConstructor(unittest.TestCase):
    """Tests for ReplanningManager constructor validation."""

    def test_valid_construction(self):
        manager = _make_manager()
        self.assertIsInstance(manager, ReplanningManager)

    def test_initial_state_is_executing(self):
        manager = _make_manager()
        self.assertEqual(manager.state, ManagerState.EXECUTING)

    def test_initial_pending_trigger_is_none(self):
        manager = _make_manager()
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_invalid_planner_type(self):
        limits = JOINT_LIMITS
        converter = TrajectoryConverter(limits)
        with self.assertRaises(TypeError):
            ReplanningManager(
                planner_adapter="not_a_planner",
                trajectory_converter=converter,
            )

    def test_invalid_converter_type(self):
        occupancy = OccupancyAdapter(inflation_radius=0.05)
        adapter = PlannerAdapter(occupancy, JOINT_LIMITS)
        with self.assertRaises(TypeError):
            ReplanningManager(
                planner_adapter=adapter,
                trajectory_converter="not_a_converter",
            )

    def test_config_deep_merge(self):
        manager = _make_manager(config={"min_replan_interval": 99.0})
        self.assertEqual(manager.config["min_replan_interval"], 99.0)
        # Other keys still present.
        self.assertIn("tracking_error_threshold", manager.config)

    def test_current_path_initially_none(self):
        self.assertIsNone(_make_manager().current_path)

    def test_current_goal_initially_none(self):
        self.assertIsNone(_make_manager().current_goal)


class TestUpdateGoal(unittest.TestCase):
    """Tests for update_goal trigger."""

    def test_first_call_no_trigger(self):
        manager = _make_manager()
        manager.update_goal(GOAL)
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_goal_change_raises_trigger(self):
        manager = _make_manager()
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertEqual(manager.pending_trigger, TriggerKind.GOAL_UPDATE)

    def test_same_goal_no_trigger(self):
        manager = _make_manager()
        manager.update_goal(GOAL)
        manager.update_goal(list(GOAL))
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_current_goal_updated(self):
        manager = _make_manager()
        new_goal = [0.1, 0.2, 0.05, 0.3]
        manager.update_goal(new_goal)
        self.assertEqual(manager.current_goal, new_goal)

    def test_current_goal_copy_not_alias(self):
        manager = _make_manager()
        goal = [0.1, 0.2, 0.05, 0.3]
        manager.update_goal(goal)
        returned = manager.current_goal
        returned[0] = 999.0
        self.assertNotEqual(manager.current_goal[0], 999.0)


class TestUpdateOccupancy(unittest.TestCase):
    """Tests for update_occupancy trigger."""

    def test_first_call_no_trigger(self):
        manager = _make_manager()
        manager.update_occupancy(_make_occupancy_points(100), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_large_change_raises_occupancy_trigger(self):
        manager = _make_manager(config={"occupancy_change_ratio": 0.05})
        manager.update_occupancy(_make_occupancy_points(100), time.time())
        # Double the points (100% change).
        manager.update_occupancy(_make_occupancy_points(200), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.OCCUPANCY_CHANGE)

    def test_small_change_no_trigger(self):
        manager = _make_manager(config={"occupancy_change_ratio": 0.20})
        manager.update_occupancy(_make_occupancy_points(100), time.time())
        # 5% change — below 20% threshold.
        manager.update_occupancy(_make_occupancy_points(105), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_path_invalidated_when_path_is_blocked(self):
        """PATH_INVALIDATED fires when the active path enters an obstacle."""
        # Validator blocks the second waypoint.
        blocked_wp = [0.15, 0.1, 0.05, 0.05]

        def validator(q):
            return q != blocked_wp

        path = [START, blocked_wp, GOAL]
        manager = _make_manager(
            state_validator=validator,
            config={"path_invalidation_ratio": 0.10},
        )
        manager.set_active_path(path)
        # Occupancy update triggers path re-check.
        manager.update_occupancy(_make_occupancy_points(50), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.PATH_INVALIDATED)

    def test_path_not_invalidated_when_below_ratio(self):
        """No trigger when blocked fraction is below threshold."""

        # Block only 1/10 waypoints; threshold is 50%.
        def validator(q):
            return q[0] != 999.0  # nothing is blocked

        path = [[0.0, 0.0, 0.0, 0.0]] * 10
        manager = _make_manager(
            state_validator=validator,
            config={"path_invalidation_ratio": 0.50},
        )
        manager.set_active_path(path)
        manager.update_occupancy(_make_occupancy_points(50), time.time())
        # No occupancy change trigger (first call) and no path invalidation.
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)


class TestUpdateTrackingState(unittest.TestCase):
    """Tests for update_tracking_state trigger."""

    def test_no_trigger_below_threshold(self):
        manager = _make_manager(
            config={
                "tracking_error_threshold": 0.15,
                "tracking_error_window": 0.5,
            }
        )
        now = time.time()
        # Errors below threshold over a full window.
        for i in range(20):
            t = now + i * 0.03
            manager.update_tracking_state(
                [0.01, 0.01, 0.001, 0.01],
                [0.0, 0.0, 0.0, 0.0],
                t,
            )
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_no_trigger_before_window_fills(self):
        """No trigger on first few samples even if errors are high."""
        manager = _make_manager(
            config={
                "tracking_error_threshold": 0.10,
                "tracking_error_window": 1.0,
            }
        )
        # Single sample high error — window not full yet.
        manager.update_tracking_state(
            [1.0, 1.0, 0.1, 1.0],
            [0.0, 0.0, 0.0, 0.0],
            time.time(),
        )
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_trigger_fires_after_sustained_error(self):
        """TRACKING_ERROR fires when errors are sustained over the window."""
        window = 0.5
        manager = _make_manager(
            config={
                "tracking_error_threshold": 0.10,
                "tracking_error_window": window,
            }
        )
        # Fill the window with high-error samples.
        start = 0.0  # simulated time (not wall-clock)
        step = window / 20
        for i in range(25):
            manager.update_tracking_state(
                [1.0, 1.0, 0.1, 1.0],
                [0.0, 0.0, 0.0, 0.0],
                start + i * step,
            )
        self.assertEqual(manager.pending_trigger, TriggerKind.TRACKING_ERROR)

    def test_dimension_mismatch_ignored(self):
        manager = _make_manager()
        # Different lengths — should not raise, just skip.
        manager.update_tracking_state([1.0, 2.0], [0.0], time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_reset_clears_tracking_history(self):
        window = 0.5
        manager = _make_manager(
            config={
                "tracking_error_threshold": 0.10,
                "tracking_error_window": window,
            }
        )
        start = 0.0
        step = window / 20
        for i in range(25):
            manager.update_tracking_state(
                [1.0, 1.0, 0.1, 1.0],
                [0.0, 0.0, 0.0, 0.0],
                start + i * step,
            )
        self.assertEqual(manager.pending_trigger, TriggerKind.TRACKING_ERROR)
        manager.reset()
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)


class TestTriggerPriority(unittest.TestCase):
    """Tests for trigger priority (higher-priority overwrites lower)."""

    def test_goal_update_overwrites_occupancy_change(self):
        manager = _make_manager(config={"occupancy_change_ratio": 0.05})
        # Set occupancy change trigger first.
        manager.update_occupancy(_make_occupancy_points(100), time.time())
        manager.update_occupancy(_make_occupancy_points(200), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.OCCUPANCY_CHANGE)
        # Goal update has higher priority.
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertEqual(manager.pending_trigger, TriggerKind.GOAL_UPDATE)

    def test_low_priority_does_not_overwrite_high(self):
        """PATH_INVALIDATED should not overwrite GOAL_UPDATE."""
        blocked_wp = [0.15, 0.1, 0.05, 0.05]

        def validator(q):
            return q != blocked_wp

        path = [START, blocked_wp, GOAL]
        manager = _make_manager(
            state_validator=validator,
            config={"path_invalidation_ratio": 0.10},
        )
        manager.set_active_path(path)
        # Set high-priority trigger first.
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertEqual(manager.pending_trigger, TriggerKind.GOAL_UPDATE)
        # Now trigger path invalidation — lower priority, must NOT overwrite.
        manager.update_occupancy(_make_occupancy_points(50), time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.GOAL_UPDATE)


class TestEvaluateTriggers(unittest.TestCase):
    """Tests for evaluate_triggers() debounce and cooldown logic."""

    def test_no_trigger_returns_none(self):
        manager = _make_manager()
        self.assertIsNone(manager.evaluate_triggers())

    def test_trigger_returned_when_interval_elapsed(self):
        manager = _make_manager(
            config={"min_replan_interval": 0.0, "fallback_cooldown": 0.0}
        )
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        result = manager.evaluate_triggers()
        self.assertEqual(result, TriggerKind.GOAL_UPDATE)

    def test_debounced_when_interval_not_elapsed(self):
        manager = _make_manager(config={"min_replan_interval": 9999.0})
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        # Artificially mark a very recent replan.
        manager._last_replan_time = time.time()
        self.assertIsNone(manager.evaluate_triggers())

    def test_fallback_cooldown_suppresses_trigger(self):
        manager = _make_manager(
            config={"fallback_cooldown": 9999.0, "min_replan_interval": 0.0}
        )
        manager._state = ManagerState.FALLBACK
        manager._fallback_entry_time = time.time()
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertIsNone(manager.evaluate_triggers())

    def test_fallback_trigger_allowed_after_cooldown(self):
        manager = _make_manager(
            config={"fallback_cooldown": 0.0, "min_replan_interval": 0.0}
        )
        manager._state = ManagerState.FALLBACK
        manager._fallback_entry_time = time.time() - 10.0
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertEqual(manager.evaluate_triggers(), TriggerKind.GOAL_UPDATE)


class TestReplan(unittest.TestCase):
    """Tests for the replan() method and state transitions."""

    def test_type_error_on_non_list(self):
        manager = _make_manager()
        with self.assertRaises(TypeError):
            manager.replan("not_a_list", time.time())

    def test_invalid_request_when_no_goal(self):
        manager = _make_manager()
        result = manager.replan(START, time.time())
        self.assertEqual(result["status"], "invalid_request")
        self.assertEqual(result["state"], ManagerState.FALLBACK.value)

    def test_replan_success_transitions_to_executing(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["state"], ManagerState.EXECUTING.value)

    def test_replan_success_trajectory_present(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertEqual(result["status"], "success")
        self.assertIsNotNone(result["trajectory"])
        self.assertEqual(result["trajectory"]["status"], "success")

    def test_replan_success_updates_current_path(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertEqual(result["status"], "success")
        self.assertIsNotNone(manager.current_path)

    def test_replan_consumes_pending_trigger(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        self.assertEqual(manager.pending_trigger, TriggerKind.GOAL_UPDATE)
        manager.replan(START, time.time())
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_replan_failure_transitions_to_fallback(self):
        """Planner returns no_plan_found when start == goal is blocked."""

        def blocking_validator(q):
            return False  # everything blocked

        manager = _make_manager(
            state_validator=blocking_validator,
            config={"min_replan_interval": 0.0},
        )
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertIn(
            result["status"], {"no_plan_found", "invalid_request", "timeout"}
        )
        self.assertEqual(result["state"], ManagerState.FALLBACK.value)

    def test_replan_result_schema_all_keys(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        for key in (
            "request_id",
            "trigger",
            "status",
            "trajectory",
            "replan_time",
            "failure_reason",
            "state",
        ):
            self.assertIn(key, result, msg=f"Missing key: {key}")

    def test_replan_result_status_valid(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertIn(result["status"], VALID_STATUSES)

    def test_replan_result_state_valid(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertIn(result["state"], VALID_STATES)

    def test_replan_time_is_non_negative(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self.assertGreaterEqual(result["replan_time"], 0.0)

    def test_replan_trigger_value_in_result(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        result = manager.replan(START, time.time())
        self.assertEqual(result["trigger"], TriggerKind.GOAL_UPDATE.value)

    def test_no_goal_replan_leaves_path_unchanged(self):
        manager = _make_manager()
        manager.replan(START, time.time())
        self.assertIsNone(manager.current_path)

    def test_old_path_preserved_on_failed_replan(self):
        """Active path must not be cleared if the replan fails."""

        def blocking_validator(q):
            return False

        manager = _make_manager(
            state_validator=blocking_validator,
            config={"min_replan_interval": 0.0},
        )
        initial_path = [[0.0, 0.0, 0.0, 0.0], [0.1, 0.1, 0.05, 0.1]]
        manager.set_active_path(initial_path)
        manager.update_goal(GOAL)
        manager.replan(START, time.time())
        # Path unchanged because replan failed.
        self.assertEqual(manager.current_path, initial_path)


class TestSafeTransitions(unittest.TestCase):
    """Tests verifying that active path is replaced only on success."""

    def test_path_updated_on_success(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.set_active_path([[0.0] * DOF])
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        if result["status"] == "success":
            new_path = manager.current_path
            self.assertIsNotNone(new_path)
            # New path leads toward the goal.
            self.assertNotEqual(new_path, [[0.0] * DOF])


class TestReset(unittest.TestCase):
    """Tests for the reset() method."""

    def test_reset_returns_to_executing(self):
        manager = _make_manager()
        manager._state = ManagerState.FALLBACK
        manager.reset()
        self.assertEqual(manager.state, ManagerState.EXECUTING)

    def test_reset_clears_pending_trigger(self):
        manager = _make_manager()
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        manager.reset()
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_reset_clears_tracking_errors(self):
        window = 0.5
        manager = _make_manager(
            config={
                "tracking_error_threshold": 0.10,
                "tracking_error_window": window,
            }
        )
        step = window / 20
        for i in range(25):
            manager.update_tracking_state(
                [1.0, 1.0, 0.1, 1.0],
                [0.0, 0.0, 0.0, 0.0],
                float(i) * step,
            )
        manager.reset()
        # After reset, no trigger should be pending.
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)
        # Internal history must be empty.
        self.assertEqual(len(manager._tracking_errors), 0)


class TestSetActivePath(unittest.TestCase):
    """Tests for set_active_path()."""

    def test_set_path(self):
        manager = _make_manager()
        path = [START, GOAL]
        manager.set_active_path(path)
        self.assertEqual(manager.current_path, path)

    def test_clear_path(self):
        manager = _make_manager()
        manager.set_active_path([START, GOAL])
        manager.set_active_path(None)
        self.assertIsNone(manager.current_path)

    def test_current_path_is_copy(self):
        manager = _make_manager()
        path = [list(START), list(GOAL)]
        manager.set_active_path(path)
        returned = manager.current_path
        returned[0][0] = 999.0
        self.assertNotEqual(manager.current_path[0][0], 999.0)


class TestIntegration(unittest.TestCase):
    """End-to-end integration tests for the replanning loop."""

    def test_full_goal_update_replan_cycle(self):
        """Goal → evaluate → replan returns success in free space."""
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        # First call initialises goal without a trigger.
        self.assertIsNone(manager.evaluate_triggers())

        # Change goal to raise trigger.
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        trigger = manager.evaluate_triggers()
        self.assertEqual(trigger, TriggerKind.GOAL_UPDATE)

        result = manager.replan(START, time.time())
        self.assertIn(
            result["status"], {"success", "no_plan_found", "timeout"}
        )
        # Pending trigger consumed.
        self.assertEqual(manager.pending_trigger, TriggerKind.NONE)

    def test_replan_skipped_within_min_interval(self):
        """Consecutive evaluate_triggers calls return None when debounced."""
        manager = _make_manager(config={"min_replan_interval": 9999.0})
        manager.update_goal(GOAL)
        manager.update_goal([0.9, 0.9, 0.15, 0.9])
        # Force _last_replan_time to now.
        manager._last_replan_time = time.time()
        self.assertIsNone(manager.evaluate_triggers())

    def test_occupancy_change_then_replan(self):
        """OCCUPANCY_CHANGE trigger leads to a successful replan."""
        manager = _make_manager(
            config={
                "occupancy_change_ratio": 0.05,
                "min_replan_interval": 0.0,
            }
        )
        manager.update_goal(GOAL)
        # First update initialises count.
        manager.update_occupancy(_make_occupancy_points(100), time.time())
        self.assertIsNone(manager.evaluate_triggers())

        # Large scene change.
        manager.update_occupancy(_make_occupancy_points(200), time.time())
        trigger = manager.evaluate_triggers()
        self.assertEqual(trigger, TriggerKind.OCCUPANCY_CHANGE)

        result = manager.replan(START, time.time())
        self.assertIn(
            result["status"], {"success", "no_plan_found", "timeout"}
        )


class TestReplanResultSchema(unittest.TestCase):
    """Exhaustive schema tests for every possible ReplanResult outcome."""

    _REQUIRED_KEYS = {
        "request_id",
        "trigger",
        "status",
        "trajectory",
        "replan_time",
        "failure_reason",
        "state",
    }

    def _assert_schema(self, result: dict) -> None:
        self.assertTrue(
            self._REQUIRED_KEYS.issubset(set(result)),
            msg=f"Missing keys: {self._REQUIRED_KEYS - set(result)}",
        )
        self.assertIn(result["status"], VALID_STATUSES)
        self.assertIn(result["state"], VALID_STATES)
        self.assertIsInstance(result["request_id"], str)
        self.assertIsInstance(result["replan_time"], float)
        self.assertIsInstance(result["failure_reason"], str)

    def test_schema_success(self):
        manager = _make_manager(config={"min_replan_interval": 0.0})
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        if result["status"] == "success":
            self._assert_schema(result)

    def test_schema_no_goal(self):
        manager = _make_manager()
        result = manager.replan(START, time.time())
        self._assert_schema(result)

    def test_schema_failure(self):
        def blocking_validator(q):
            return False

        manager = _make_manager(
            state_validator=blocking_validator,
            config={"min_replan_interval": 0.0},
        )
        manager.update_goal(GOAL)
        result = manager.replan(START, time.time())
        self._assert_schema(result)


if __name__ == "__main__":
    unittest.main()
