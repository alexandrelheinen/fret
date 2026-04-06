"""Replanning manager for ARCO-FRET scene-update and deviation recovery (Issue 07).

Provides :class:`ReplanningManager`, the single entry point for reactive
replanning decisions in FRET.  The manager sits above
:class:`~fret.planning.PlannerAdapter` and
:class:`~fret.planning.TrajectoryConverter`, evaluating replanning triggers
(occupancy change, path invalidation, tracking error, goal update) with
configurable hysteresis and a global debounce interval to prevent oscillation.
Safe trajectory transitions are guaranteed by keeping the previous path active
until a new plan is successfully converted.

**Design invariants:**

- The manager is decoupled from the ROS 2 runtime; callers inject occupancy
  snapshots, tracking state, and goal updates explicitly.
- All configurable thresholds live in ``config/replanning.yaml`` and are
  mirrored in :data:`DEFAULT_CONFIG`.
- The manager never raises on expected failure conditions (no-solution,
  timeout, debounced).  All outcomes are represented in the
  :ref:`ReplanResult <replan-result>` dict.
- Replanning is debounced by a global ``min_replan_interval`` and an
  optional ``fallback_cooldown`` to prevent uncontrolled oscillation.
- A ``TRACKING_ERROR`` trigger requires the RMS error to remain above the
  threshold for a sustained ``tracking_error_window`` (hysteresis).
- The active path is replaced only after a successful plan-and-convert cycle,
  ensuring the robot never references an invalid trajectory.

**State machine:**

.. code-block:: text

    ┌───────────┐  trigger (not debounced)  ┌────────────┐
    │ EXECUTING ├──────────────────────────►│ REPLANNING │
    └───────────┘                           └─────┬──────┘
          ▲  replan success                       │
          └───────────────────────────────────────┘
          ┌──────────────────── replan failure ───┘
          ▼
    ┌──────────┐  trigger after cooldown  ┌────────────┐
    │ FALLBACK ├─────────────────────────►│ REPLANNING │
    └──────────┘                          └────────────┘
          ▲
          │  reset()
          └──────────────────── (any state)

**Trigger priority (highest wins when multiple are pending):**

1. ``GOAL_UPDATE``       — goal position changed
2. ``TRACKING_ERROR``    — sustained RMS tracking error above threshold
3. ``PATH_INVALIDATED``  — fraction of path waypoints blocked by occupancy
4. ``OCCUPANCY_CHANGE``  — scene change ratio exceeded threshold

**ReplanResult dict schema:**

- ``request_id``    — Fresh UUID string for this replan attempt.
- ``trigger``       — :class:`TriggerKind` value that initiated the replan.
- ``status``        — ``"success"`` | ``"no_plan_found"`` | ``"timeout"``
  | ``"invalid_request"`` | ``"debounced"``.
- ``trajectory``    — ``TrajectoryResult`` dict on success; ``None`` otherwise.
- ``replan_time``   — Wall-clock duration of the replan call (seconds,
  monotonic clock).
- ``failure_reason`` — Human-readable explanation; empty string on success.
- ``state``         — :class:`ManagerState` value after the transition.

See also:
    ``docs/arco/issue-07-replanning-triggers-and-scene-update-loop.md`` —
    full specification and acceptance criteria.
"""

from __future__ import annotations

import logging
import math
import time
import uuid
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from fret.planning.planner_adapter import PlannerAdapter
from fret.planning.trajectory_converter import TrajectoryConverter

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

#: Valid states for the replanning state machine.
VALID_STATES: frozenset[str] = frozenset(
    {"executing", "replanning", "fallback"}
)

#: Valid status values for a :class:`ReplanResult`.
VALID_STATUSES: frozenset[str] = frozenset(
    {
        "success",
        "no_plan_found",
        "timeout",
        "invalid_request",
        "debounced",
    }
)

#: Default configuration (mirrors ``config/replanning.yaml``).
DEFAULT_CONFIG: Dict[str, Any] = {
    # Minimum seconds between two successive replans (global debounce).
    "min_replan_interval": 1.0,
    # Fraction of path waypoints that must be invalid before a
    # PATH_INVALIDATED trigger is raised (0.0–1.0 exclusive).
    "path_invalidation_ratio": 0.10,
    # RMS joint-space tracking error (rad/m) above which a replan is
    # triggered (subject to tracking_error_window hysteresis).
    "tracking_error_threshold": 0.15,
    # Duration (seconds) for which tracking error must continuously
    # exceed tracking_error_threshold before a trigger is raised.
    "tracking_error_window": 0.5,
    # Fraction of occupancy point count that must change between two
    # consecutive snapshots before an OCCUPANCY_CHANGE trigger is raised
    # (0.0–1.0 exclusive).
    "occupancy_change_ratio": 0.05,
    # Seconds the manager stays in FALLBACK before another replan attempt
    # is allowed (fallback cooldown).
    "fallback_cooldown": 2.0,
}


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class ManagerState(str, Enum):
    """Finite states of the replanning state machine.

    States are exposed as plain strings so that they can be stored directly
    in ``ReplanResult["state"]`` without requiring enum imports in callers.
    """

    EXECUTING = "executing"
    REPLANNING = "replanning"
    FALLBACK = "fallback"


class TriggerKind(str, Enum):
    """Replanning trigger identifiers.

    Trigger kinds are also exposed as plain strings via the ``value``
    attribute, matching the ``ReplanResult["trigger"]`` field.

    Priority order (highest first):

    1. ``GOAL_UPDATE``
    2. ``TRACKING_ERROR``
    3. ``PATH_INVALIDATED``
    4. ``OCCUPANCY_CHANGE``
    5. ``NONE``
    """

    NONE = "none"
    OCCUPANCY_CHANGE = "occupancy_change"
    PATH_INVALIDATED = "path_invalidated"
    TRACKING_ERROR = "tracking_error"
    GOAL_UPDATE = "goal_update"


# Fraction of the tracking-error window that must be covered by retained
# samples before the hysteresis check fires.  A value of 1.0 would require
# exactly window-duration of history, which is impossible with discrete samples
# and float timestamps; 0.9 tolerates small gaps at the window boundary while
# still requiring a sustained violation rather than a momentary spike.
_WINDOW_FILL_RATIO: float = 0.9

# Lookup table: higher number → higher priority.
_TRIGGER_PRIORITY: Dict[TriggerKind, int] = {
    TriggerKind.NONE: 0,
    TriggerKind.OCCUPANCY_CHANGE: 1,
    TriggerKind.PATH_INVALIDATED: 2,
    TriggerKind.TRACKING_ERROR: 3,
    TriggerKind.GOAL_UPDATE: 4,
}


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _rms_error(
    actual: List[float],
    reference: List[float],
) -> float:
    """Compute the root-mean-square error between two joint-space vectors.

    Args:
        actual: Measured joint positions (radians or metres).
        reference: Reference (desired) joint positions.

    Returns:
        RMS error as a non-negative float; ``0.0`` when both vectors are
        empty.
    """
    n = len(actual)
    if n == 0:
        return 0.0
    squared_sum = sum((a - r) ** 2 for a, r in zip(actual, reference))
    return math.sqrt(squared_sum / n)


def _free_space_validator(
    joint_positions: List[float],  # noqa: ARG001 – intentional no-op
) -> bool:
    """Default state validator: always returns ``True`` (no obstacles).

    This is a no-op placeholder used when no collision-checking validator is
    supplied at construction.  It models a completely obstacle-free workspace
    and is intended for testing and simulation scenarios only.

    Args:
        joint_positions: Joint configuration (unused in this implementation).

    Returns:
        Always ``True``.
    """
    return True


def _deep_merge(
    base: Dict[str, Any],
    override: Dict[str, Any],
) -> Dict[str, Any]:
    """Recursively merge *override* into a copy of *base*.

    Nested dicts are merged recursively; all other values in *override*
    replace those in *base*.

    Args:
        base: Default configuration dict.
        override: Caller-supplied overrides.

    Returns:
        New merged dict; *base* is not mutated.
    """
    result: Dict[str, Any] = dict(base)
    for key, val in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(val, dict)
        ):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = val
    return result


# ---------------------------------------------------------------------------
# ReplanningManager
# ---------------------------------------------------------------------------


class ReplanningManager:
    """Manages replanning triggers and safe trajectory transitions.

    The manager wraps a :class:`~fret.planning.PlannerAdapter` and a
    :class:`~fret.planning.TrajectoryConverter` and adds:

    - **Trigger detection** – occupancy change, path invalidation, tracking
      error (with hysteresis window), and goal update.
    - **Debounce** – a ``min_replan_interval`` prevents replanning faster
      than a configurable rate, and a ``fallback_cooldown`` limits recovery
      attempts after planning failures.
    - **State machine** – ``EXECUTING → REPLANNING → EXECUTING`` (success)
      or ``REPLANNING → FALLBACK`` (failure), with ``reset()`` returning any
      state to ``EXECUTING``.
    - **Safe transitions** – the active path is replaced only after a
      successful plan-and-convert cycle.
    - **Structured logging** – every trigger source and outcome is logged via
      the module-level ``logging.Logger``.

    Args:
        planner_adapter: Configured :class:`~fret.planning.PlannerAdapter`.
        trajectory_converter: Configured
            :class:`~fret.planning.TrajectoryConverter`.
        state_validator: Optional callable
            ``(joint_positions: list[float]) -> bool`` returning ``True``
            when a configuration is collision-free.  Used for path
            invalidation checks.  Defaults to a free-space validator (always
            ``True``).
        config: Optional configuration dict overriding :data:`DEFAULT_CONFIG`.

    Raises:
        TypeError: If *planner_adapter* or *trajectory_converter* is not the
            expected type.

    Example::

        import math
        from fret.perception import OccupancyAdapter
        from fret.planning import PlannerAdapter, TrajectoryConverter
        from fret.planning.replanning_manager import ReplanningManager

        limits = [(-math.pi, math.pi)] * 4
        occupancy = OccupancyAdapter(inflation_radius=0.05)
        adapter = PlannerAdapter(occupancy, limits)
        converter = TrajectoryConverter(limits)

        manager = ReplanningManager(adapter, converter)
        manager.update_goal([0.5, 0.3, 0.1, 0.2])

        trigger = manager.evaluate_triggers()
        if trigger is not None:
            result = manager.replan([0.0, 0.0, 0.0, 0.0], time.time())
            if result["status"] == "success":
                trajectory = result["trajectory"]
    """

    def __init__(
        self,
        planner_adapter: PlannerAdapter,
        trajectory_converter: TrajectoryConverter,
        state_validator: Optional[Callable[[List[float]], bool]] = None,
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not isinstance(planner_adapter, PlannerAdapter):
            raise TypeError(
                "planner_adapter must be a PlannerAdapter instance, "
                f"got {type(planner_adapter).__name__}"
            )
        if not isinstance(trajectory_converter, TrajectoryConverter):
            raise TypeError(
                "trajectory_converter must be a TrajectoryConverter instance, "
                f"got {type(trajectory_converter).__name__}"
            )

        self._planner = planner_adapter
        self._converter = trajectory_converter
        self._state_validator: Callable[[List[float]], bool] = (
            state_validator
            if state_validator is not None
            else _free_space_validator
        )

        self._config: Dict[str, Any] = _deep_merge(
            DEFAULT_CONFIG, config or {}
        )

        # State machine.
        self._state: ManagerState = ManagerState.EXECUTING

        # Pending trigger (highest-priority wins).
        self._pending_trigger: TriggerKind = TriggerKind.NONE

        # Debounce timestamps (POSIX / time.time() source for monotone comparisons).
        self._last_replan_time: float = -math.inf
        self._fallback_entry_time: float = -math.inf

        # Occupancy change tracking: previous point count.
        self._prev_occupancy_count: int = 0

        # Current active path (updated only on successful replan).
        self._current_path: Optional[List[List[float]]] = None

        # Current goal joint positions.
        self._current_goal: Optional[List[float]] = None

        # Tracking error history: list of (POSIX timestamp, RMS error) pairs.
        self._tracking_errors: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> ManagerState:
        """Current state machine state."""
        return self._state

    @property
    def pending_trigger(self) -> TriggerKind:
        """Highest-priority pending trigger (``NONE`` if quiescent)."""
        return self._pending_trigger

    @property
    def current_path(self) -> Optional[List[List[float]]]:
        """A copy of the current active path, or ``None`` if not set."""
        if self._current_path is None:
            return None
        return [list(wp) for wp in self._current_path]

    @property
    def current_goal(self) -> Optional[List[float]]:
        """A copy of the current goal joint positions, or ``None``."""
        return (
            list(self._current_goal)
            if self._current_goal is not None
            else None
        )

    @property
    def config(self) -> Dict[str, Any]:
        """Resolved configuration (merged with :data:`DEFAULT_CONFIG`)."""
        return dict(self._config)

    # ------------------------------------------------------------------
    # Mutators
    # ------------------------------------------------------------------

    def set_active_path(
        self,
        path: Optional[List[List[float]]],
    ) -> None:
        """Register the currently executing path for invalidation checks.

        Call this after a successful initial plan so that the manager can
        detect when new occupancy data blocks existing waypoints.

        Args:
            path: Ordered list of joint configurations (waypoints), or
                ``None`` to clear the stored path.
        """
        self._current_path = (
            [list(wp) for wp in path] if path is not None else None
        )

    # ------------------------------------------------------------------
    # Trigger update methods
    # ------------------------------------------------------------------

    def update_occupancy(
        self,
        new_occupancy_points: List[Tuple[float, float, float]],
        stamp: float,
    ) -> None:
        """Notify the manager of a new occupancy snapshot.

        Compares the point count against the previous snapshot to detect
        significant scene changes.  Also checks whether the current active
        path has been invalidated by the new occupancy (requires a
        ``state_validator`` supplied at construction).

        Args:
            new_occupancy_points: List of 3-D obstacle points
                ``(x, y, z)`` from the latest sensor snapshot.
            stamp: POSIX timestamp of the snapshot (``time.time()``
                clock source), forwarded to the planner on replan.
        """
        new_count = len(new_occupancy_points)
        prev_count = self._prev_occupancy_count

        if prev_count > 0:
            change_ratio = abs(new_count - prev_count) / prev_count
            threshold: float = self._config["occupancy_change_ratio"]
            if change_ratio > threshold:
                self._raise_trigger(TriggerKind.OCCUPANCY_CHANGE)
                _LOG.debug(
                    "OCCUPANCY_CHANGE: count %d → %d (ratio %.3f > %.3f)",
                    prev_count,
                    new_count,
                    change_ratio,
                    threshold,
                )

        self._prev_occupancy_count = new_count

        # Path invalidation check after occupancy update.
        if self._current_path is not None:
            self._check_path_validity()

    def update_tracking_state(
        self,
        joint_positions: List[float],
        reference_positions: List[float],
        timestamp: float,
    ) -> None:
        """Record the current tracking error sample.

        A ``TRACKING_ERROR`` trigger is raised only when the RMS error has
        continuously exceeded ``tracking_error_threshold`` over the full
        ``tracking_error_window`` (hysteresis).

        Args:
            joint_positions: Measured joint positions (rad/m).
            reference_positions: Reference (desired) joint positions.
            timestamp: POSIX timestamp of this sample (``time.time()``
                clock source).
        """
        if len(joint_positions) != len(reference_positions):
            _LOG.warning(
                "update_tracking_state: dimension mismatch %d vs %d; skipped",
                len(joint_positions),
                len(reference_positions),
            )
            return

        rms = _rms_error(joint_positions, reference_positions)
        self._tracking_errors.append((timestamp, rms))

        # Prune entries older than the hysteresis window.
        window: float = self._config["tracking_error_window"]
        cutoff = timestamp - window
        self._tracking_errors = [
            (t, e) for (t, e) in self._tracking_errors if t >= cutoff
        ]

        if not self._tracking_errors:
            return

        # The trigger fires only when the window is "full" and ALL samples
        # exceed the threshold — the hysteresis property.
        oldest_t = self._tracking_errors[0][0]
        window_full = (timestamp - oldest_t) >= window * _WINDOW_FILL_RATIO
        threshold: float = self._config["tracking_error_threshold"]

        if window_full and all(
            e > threshold for (_, e) in self._tracking_errors
        ):
            avg_e = sum(e for (_, e) in self._tracking_errors) / len(
                self._tracking_errors
            )
            self._raise_trigger(TriggerKind.TRACKING_ERROR)
            _LOG.debug(
                "TRACKING_ERROR: sustained RMS %.4f > %.4f over %.2f s window",
                avg_e,
                threshold,
                window,
            )

    def update_goal(
        self,
        goal_joint_positions: List[float],
    ) -> None:
        """Notify the manager of a goal change.

        A ``GOAL_UPDATE`` trigger is raised immediately when the new goal
        differs from the current goal.  The first call to this method
        initialises the goal without raising a trigger.

        Args:
            goal_joint_positions: New goal joint positions (rad/m).
        """
        if self._current_goal is None:
            self._current_goal = list(goal_joint_positions)
            _LOG.debug(
                "Goal initialised: %s",
                goal_joint_positions,
            )
            return

        if goal_joint_positions != self._current_goal:
            self._current_goal = list(goal_joint_positions)
            self._raise_trigger(TriggerKind.GOAL_UPDATE)
            _LOG.debug("GOAL_UPDATE: new goal %s", goal_joint_positions)

    # ------------------------------------------------------------------
    # Trigger evaluation
    # ------------------------------------------------------------------

    def evaluate_triggers(self) -> Optional[TriggerKind]:
        """Evaluate whether a replan should be triggered now.

        Applies the global debounce (``min_replan_interval``) and the
        fallback cooldown.  Returns ``None`` when no replan should be
        attempted, even if a trigger is pending.

        Returns:
            The highest-priority pending :class:`TriggerKind` when
            replanning is warranted, or ``None`` when suppressed by
            debounce or fallback cooldown.
        """
        if self._pending_trigger == TriggerKind.NONE:
            return None

        now = time.time()

        # Fallback cooldown: avoid rapid retry after planning failure.
        if self._state == ManagerState.FALLBACK:
            cooldown: float = self._config["fallback_cooldown"]
            elapsed = now - self._fallback_entry_time
            if elapsed < cooldown:
                _LOG.debug(
                    "Trigger %s suppressed (fallback cooldown, %.1f s remaining)",
                    self._pending_trigger.value,
                    cooldown - elapsed,
                )
                return None

        # Global replan debounce.
        min_interval: float = self._config["min_replan_interval"]
        elapsed_since_last = now - self._last_replan_time
        if elapsed_since_last < min_interval:
            _LOG.debug(
                "Trigger %s suppressed (debounce, %.1f s remaining)",
                self._pending_trigger.value,
                min_interval - elapsed_since_last,
            )
            return None

        return self._pending_trigger

    # ------------------------------------------------------------------
    # Replanning
    # ------------------------------------------------------------------

    def replan(
        self,
        current_joint_positions: List[float],
        occupancy_stamp: float,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Execute a replanning request and transition the state machine.

        Delegates to the injected :class:`~fret.planning.PlannerAdapter`
        and, on successful planning, immediately converts the path with
        :class:`~fret.planning.TrajectoryConverter`.  The active path is
        replaced only when both steps succeed.

        The trigger consumed is the current :attr:`pending_trigger`.  If no
        trigger is pending the replan is executed unconditionally (useful for
        forced replans such as goal updates).

        Args:
            current_joint_positions: Robot's current joint positions used
                as the planning start configuration.
            occupancy_stamp: POSIX timestamp of the occupancy snapshot
                forwarded to the planner (``time.time()`` clock source).
            timeout: Optional planning timeout override (seconds).  When
                ``None``, the planner adapter's ``default_timeout`` is used.

        Returns:
            ``ReplanResult`` dict with keys: ``request_id``, ``trigger``,
            ``status``, ``trajectory``, ``replan_time``, ``failure_reason``,
            ``state``.

        Raises:
            TypeError: If *current_joint_positions* is not a list.
        """
        if not isinstance(current_joint_positions, list):
            raise TypeError(
                "current_joint_positions must be a list, "
                f"got {type(current_joint_positions).__name__}"
            )

        trigger = self._pending_trigger
        t_start = time.monotonic()

        # Consume the pending trigger and record the replan time.
        self._pending_trigger = TriggerKind.NONE
        self._last_replan_time = time.time()

        # Guard: a goal is required.
        if self._current_goal is None:
            reason = "no goal set; call update_goal() before replan()"
            _LOG.warning(
                "Replan failed (trigger=%s): %s",
                trigger.value,
                reason,
            )
            self._state = ManagerState.FALLBACK
            self._fallback_entry_time = time.time()
            return self._build_result(
                trigger=trigger,
                status="invalid_request",
                trajectory=None,
                replan_time=time.monotonic() - t_start,
                failure_reason=reason,
            )

        # Transition to REPLANNING.
        self._state = ManagerState.REPLANNING
        _LOG.info(
            "Replanning initiated (trigger=%s, start=%s, goal=%s)",
            trigger.value,
            current_joint_positions,
            self._current_goal,
        )

        # Build and submit the planning request.
        effective_timeout: float = (
            timeout
            if timeout is not None
            else float(self._planner.config.get("default_timeout", 5.0))
        )
        plan_request: Dict[str, Any] = {
            "request_id": str(uuid.uuid4()),
            "start_joint_positions": list(current_joint_positions),
            "goal_joint_positions": list(self._current_goal),
            "joint_count": len(current_joint_positions),
            "occupancy_stamp": occupancy_stamp,
            "timeout": effective_timeout,
            "planner_config": {
                "algorithm": self._planner.config.get(
                    "algorithm", "rrt_connect"
                )
            },
            "reference_frame": "world",
        }

        plan_result: Dict[str, Any] = self._planner.plan(plan_request)

        if plan_result["status"] == "success":
            traj_result = self._converter.convert(
                plan_result["path"],
                request_id=plan_result["request_id"],
            )

            if traj_result["status"] == "success":
                # Safe transition: update active path only on full success.
                self._current_path = plan_result["path"]
                self._state = ManagerState.EXECUTING
                replan_time = time.monotonic() - t_start
                _LOG.info(
                    "Replan success (trigger=%s, %d waypoints, %.3f s)",
                    trigger.value,
                    plan_result["waypoint_count"],
                    replan_time,
                )
                return self._build_result(
                    trigger=trigger,
                    status="success",
                    trajectory=traj_result,
                    replan_time=replan_time,
                    failure_reason=None,
                )

            # Trajectory conversion failed.
            reason = f"trajectory_conversion_failed: {traj_result['failure_reason']}"
            self._state = ManagerState.FALLBACK
            self._fallback_entry_time = time.time()
            _LOG.warning(
                "Replan failed (trigger=%s): %s",
                trigger.value,
                reason,
            )
            return self._build_result(
                trigger=trigger,
                status="no_plan_found",
                trajectory=None,
                replan_time=time.monotonic() - t_start,
                failure_reason=reason,
            )

        # Planning failed.
        reason = plan_result.get("failure_reason") or plan_result["status"]
        self._state = ManagerState.FALLBACK
        self._fallback_entry_time = time.time()
        _LOG.warning(
            "Replan failed (trigger=%s, planner_status=%s): %s",
            trigger.value,
            plan_result["status"],
            reason,
        )
        return self._build_result(
            trigger=trigger,
            status=plan_result["status"],
            trajectory=None,
            replan_time=time.monotonic() - t_start,
            failure_reason=reason,
        )

    # ------------------------------------------------------------------
    # State control
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset the manager to :attr:`ManagerState.EXECUTING`.

        Clears the pending trigger and tracking error history.  Call this
        after an external recovery action (e.g., operator intervention) to
        allow normal operation to resume.
        """
        self._state = ManagerState.EXECUTING
        self._pending_trigger = TriggerKind.NONE
        self._tracking_errors = []
        _LOG.info("ReplanningManager reset to EXECUTING state")

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _raise_trigger(self, kind: TriggerKind) -> None:
        """Raise a trigger, keeping the highest-priority one.

        Args:
            kind: The new trigger to consider.
        """
        if _TRIGGER_PRIORITY[kind] > _TRIGGER_PRIORITY[self._pending_trigger]:
            self._pending_trigger = kind

    def _check_path_validity(self) -> None:
        """Check whether the current active path is still collision-free.

        Raises a ``PATH_INVALIDATED`` trigger when the fraction of blocked
        waypoints exceeds ``path_invalidation_ratio``.
        """
        if not self._current_path:
            return

        invalid_count = sum(
            1 for wp in self._current_path if not self._state_validator(wp)
        )
        ratio = invalid_count / len(self._current_path)
        threshold: float = self._config["path_invalidation_ratio"]

        if ratio > threshold:
            self._raise_trigger(TriggerKind.PATH_INVALIDATED)
            _LOG.debug(
                "PATH_INVALIDATED: %d/%d waypoints blocked (ratio %.3f > %.3f)",
                invalid_count,
                len(self._current_path),
                ratio,
                threshold,
            )

    def _build_result(
        self,
        *,
        trigger: TriggerKind,
        status: str,
        trajectory: Optional[Dict[str, Any]],
        replan_time: float,
        failure_reason: Optional[str],
    ) -> Dict[str, Any]:
        """Assemble a ReplanResult dict.

        Args:
            trigger: Trigger that initiated this replan.
            status: Outcome status string.
            trajectory: ``TrajectoryResult`` or ``None``.
            replan_time: Wall-clock duration of the call (seconds).
            failure_reason: Human-readable failure description or ``None``.

        Returns:
            Validated ``ReplanResult`` dict.
        """
        return {
            "request_id": str(uuid.uuid4()),
            "trigger": trigger.value,
            "status": status,
            "trajectory": trajectory,
            "replan_time": replan_time,
            "failure_reason": failure_reason or "",
            "state": self._state.value,
        }
