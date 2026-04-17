"""Replanning state-machine manager.

Manages replanning triggers and state transitions for the FRET planning
pipeline.  Wraps a ``PlannerNode`` and a ``TrajectoryConverter`` to provide
a higher-level interface that handles debouncing, attempt counting, and
occupancy updates.

State machine (``ManagerState``):

- ``IDLE``       — no active trajectory; waiting for ``start_execution``.
- ``EXECUTING``  — trajectory produced; execution in progress.
- ``REPLANNING`` — transient state during ``trigger_replan`` call.
- ``HALTED``     — max replan attempts exceeded; requires explicit ``reset()``.

Satisfies requirement FR-PLN-09.
"""

from __future__ import annotations

import enum
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.trajectory_converter import (
    TrajectoryConverter,
    TrajectoryResult,
)

if TYPE_CHECKING:
    from fret.planning.planner_node import PlannerNode

# ---------------------------------------------------------------------------
# Public enumerations
# ---------------------------------------------------------------------------


class ManagerState(enum.Enum):
    """Replanning manager states.

    Attributes:
        IDLE: No active trajectory; waiting for ``start_execution``.
        EXECUTING: Trajectory produced; execution in progress.
        REPLANNING: Transient state during a replan call.
        HALTED: Max replan attempts exceeded; requires ``reset()``.
    """

    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    REPLANNING = "REPLANNING"
    HALTED = "HALTED"


class TriggerKind(enum.Enum):
    """Classification of events that may trigger replanning.

    Attributes:
        OCCUPANCY_CHANGE: Obstacle geometry has changed significantly.
        PATH_INVALIDATED: Current path has been invalidated (e.g. new collision).
        TRACKING_ERROR: Controller tracking error exceeded threshold.
        GOAL_UPDATE: The target goal configuration has changed.
        MANUAL: Explicit caller-initiated replan.
    """

    OCCUPANCY_CHANGE = "OCCUPANCY_CHANGE"
    PATH_INVALIDATED = "PATH_INVALIDATED"
    TRACKING_ERROR = "TRACKING_ERROR"
    GOAL_UPDATE = "GOAL_UPDATE"
    MANUAL = "MANUAL"


# ---------------------------------------------------------------------------
# Default configuration values
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG: dict[str, Any] = {
    "tracking_error_threshold": 0.020,
    "occupancy_change_threshold": 0.050,
    "min_replan_interval": 1.0,
    "max_replan_attempts": 3,
}


# ---------------------------------------------------------------------------
# ReplanningManager
# ---------------------------------------------------------------------------


class ReplanningManager:
    """Manage replanning triggers and state transitions.

    Wraps a ``PlannerNode`` and ``TrajectoryConverter`` with debouncing,
    attempt counting, and automatic HALTED-transition on repeated failures.

    Args:
        planner_node: Pure-Python planning logic instance.
        trajectory_converter: Converts raw paths to time-parameterized
            trajectories.
        config: Optional configuration dict.  Accepted keys (all optional)
            may be nested under a ``"replanning"`` sub-key:

            - ``tracking_error_threshold`` (float): EE tracking error [m]
              that triggers replanning (default 0.02).
            - ``occupancy_change_threshold`` (float): Minimum obstacle
              displacement [m] to trigger replanning (default 0.05).
            - ``min_replan_interval`` (float): Minimum seconds between
              consecutive replan attempts / debounce window (default 1.0).
            - ``max_replan_attempts`` (int): Maximum consecutive replan
              attempts before transitioning to ``HALTED`` (default 3).
    """

    def __init__(
        self,
        planner_node: PlannerNode,
        trajectory_converter: TrajectoryConverter,
        config: dict | None = None,
    ) -> None:
        cfg = self._parse_config(config)
        self._planner = planner_node
        self._converter = trajectory_converter
        self._tracking_error_threshold: float = float(
            cfg["tracking_error_threshold"]
        )
        self._occupancy_change_threshold: float = float(
            cfg["occupancy_change_threshold"]
        )
        self._min_replan_interval: float = float(cfg["min_replan_interval"])
        self._max_replan_attempts: int = int(cfg["max_replan_attempts"])

        self._state: ManagerState = ManagerState.IDLE
        self._current_request: PlanningRequest | None = None
        self._last_replan_time: float = -float("inf")
        self._replan_attempts: int = 0
        self._last_payload: OccupancyUpdatePayload | None = None
        self._last_occ_points: npt.NDArray[np.float64] | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> ManagerState:
        """Current manager state."""
        return self._state

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start_execution(
        self, request: PlanningRequest
    ) -> TrajectoryResult | None:
        """Plan and start executing a trajectory.

        Calls ``planner_node.plan(request)`` and, on success, converts the
        raw path to a ``TrajectoryResult``.  Transitions to ``EXECUTING``
        on success.

        Args:
            request: Planning goal specifying start/goal configurations
                and timeout.

        Returns:
            The ``TrajectoryResult`` on success, or ``None`` if planning
            failed.
        """
        result = self._planner.plan(request)
        if result.status != PlanningStatus.SUCCESS:
            return None

        self._current_request = request
        self._replan_attempts = 0
        trajectory = self._converter.convert(result.path)
        self._state = ManagerState.EXECUTING
        return trajectory

    def trigger_replan(
        self,
        kind: TriggerKind,
        payload: OccupancyUpdatePayload | None = None,
    ) -> bool:
        """Trigger a replan.

        Replanning is only possible from the ``EXECUTING`` state.  Calls are
        debounced by ``min_replan_interval``; a second call within the window
        returns ``False`` without replanning.

        Args:
            kind: Classification of the triggering event.
            payload: Optional occupancy payload to update the planner with
                before replanning.

        Returns:
            ``True`` if replanning was initiated, ``False`` if debounced or
            the state does not allow replanning.
        """
        if self._state != ManagerState.EXECUTING:
            return False

        now = time.monotonic()
        if now - self._last_replan_time < self._min_replan_interval:
            return False

        self._state = ManagerState.REPLANNING
        self._last_replan_time = now
        self._replan_attempts += 1

        if self._replan_attempts > self._max_replan_attempts:
            self._state = ManagerState.HALTED
            return True

        if self._current_request is None:
            self._state = ManagerState.HALTED
            return True

        result = self._planner.plan(self._current_request)
        if result.status != PlanningStatus.SUCCESS:
            if self._replan_attempts >= self._max_replan_attempts:
                self._state = ManagerState.HALTED
            else:
                self._state = ManagerState.EXECUTING
            return True

        self._converter.convert(result.path)
        self._state = ManagerState.EXECUTING
        return True

    def report_tracking_error(self, error_m: float) -> None:
        """Report current EE tracking error; may trigger a replan.

        Args:
            error_m: End-effector tracking error magnitude in metres.
        """
        if error_m > self._tracking_error_threshold:
            self.trigger_replan(TriggerKind.TRACKING_ERROR)

    def report_occupancy_update(self, payload: OccupancyUpdatePayload) -> None:
        """Report a new occupancy update; may trigger a replan.

        Stores the payload and triggers replanning when the obstacle
        displacement exceeds ``occupancy_change_threshold``.

        Args:
            payload: Latest obstacle geometry snapshot.
        """
        should_replan = False

        if self._last_occ_points is not None:
            prev = self._last_occ_points
            curr = payload.obstacle_points
            if prev.shape == curr.shape and prev.shape[0] > 0:
                displacement = float(
                    np.mean(np.linalg.norm(curr - prev, axis=1))
                )
                if displacement > self._occupancy_change_threshold:
                    should_replan = True
            elif prev.shape != curr.shape:
                should_replan = True

        self._last_payload = payload
        self._last_occ_points = payload.obstacle_points.copy()

        if should_replan:
            self.trigger_replan(TriggerKind.OCCUPANCY_CHANGE, payload)

    def halt(self) -> None:
        """Transition to the ``HALTED`` state immediately."""
        self._state = ManagerState.HALTED

    def reset(self) -> None:
        """Reset to ``IDLE`` state, clearing attempt counters."""
        self._state = ManagerState.IDLE
        self._replan_attempts = 0
        self._last_replan_time = -float("inf")
        self._current_request = None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_config(config: dict | None) -> dict[str, Any]:
        """Merge user config with defaults.

        Args:
            config: User-supplied config dict (may contain a ``"replanning"``
                sub-key).

        Returns:
            Flat config dict with all required keys present.
        """
        base: dict[str, Any] = dict(_DEFAULT_CONFIG)
        if config is None:
            return base
        src: dict[str, Any] = config.get("replanning", config)
        for key in base:
            if key in src:
                base[key] = src[key]
        return base
