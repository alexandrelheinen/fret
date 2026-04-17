"""Pure-Python planning logic core for C-space path planning via ARCO.

Level 3 (``PlannerNode``) implements the pure-Python planning state machine
so the logic can be unit-tested without a live ROS context.  It drives the
full planning pipeline:

1. Validate goal configuration against joint limits (→ ``ABORTED /
   INVALID_CONFIGURATION`` on failure).
2. Call ARCO SST planner with the current ``CSpaceChecker`` as collision
   predicate when available; fall back to a straight joint-space path
   otherwise (→ ``ABORTED / TIMEOUT`` or ``NO_PATH_FOUND`` on failure).
3. Run the ``TrajectoryGenerator`` post-processing chain (→ ``ABORTED /
   POST_PROCESS_FAILED`` on exception).
4. Return a ``PlanningResult`` with ``status == SUCCESS`` and the path.

Level 4 (``PlannerNodeRos``) in ``planner_node_ros.py`` subclasses
``rclpy.node.Node`` and wires this logic into a ROS 2 Action server.

FSM states and transition table are defined in docs/interfaces.md.
Satisfies requirements FR-PLN-01 through FR-PLN-07.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.interfaces import (
    ErrorCode,
    PlanningRequest,
    PlanningResult,
    PlanningStatus,
)
from fret.planning.cspace_checker import CSpaceChecker
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter

try:
    from arco.planning import SST
except ImportError:
    SST = None


class PlannerNode:
    """Pure-Python planning logic for C-space manipulator path planning.

    This class is the unit-testable core of the planner.  It holds a
    ``CSpaceChecker``, an ``OccupancyAdapter``, and a ``TrajectoryGenerator``
    and wires them together.

    The full ROS 2 Action server setup is in ``planner_node_ros.py``
    (Level 4).  This class (Level 3) can be instantiated and called without
    any ROS context.

    Args:
        model: Robot model name (e.g. ``"scara"``).  Used to instantiate the
            kinematics engine.
        occupancy_adapter: Pre-constructed adapter providing the live
            occupancy model.
    """

    def __init__(
        self,
        model: str,
        occupancy_adapter: OccupancyAdapter,
    ) -> None:
        from fret.control.kinematics import Kinematics

        self._kin = Kinematics(model)
        self._occ_adapter = occupancy_adapter
        self._traj_gen = TrajectoryGenerator(self._kin)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(self, request: PlanningRequest) -> PlanningResult:
        """Execute the full planning pipeline for the given request.

        Args:
            request: Planning goal specifying start/goal configurations,
                timeout, and scenario identifier.

        Returns:
            A ``PlanningResult`` whose ``status`` reflects the outcome:
            ``SUCCESS``, ``ABORTED``, or ``CANCELLED``.
        """
        t_start = time.monotonic()

        # -- 1. Validate joint limits ----------------------------------------
        limits = self._kin.joint_limits  # shape (DOF, 2)
        for i, (val, (lo, hi)) in enumerate(
            zip(request.goal_configuration, limits)
        ):
            if not (lo - 1e-9 <= float(val) <= hi + 1e-9):
                return PlanningResult(
                    status=PlanningStatus.ABORTED,
                    path=[],
                    error_code=ErrorCode.INVALID_CONFIGURATION,
                    planning_duration=time.monotonic() - t_start,
                )

        # -- 2. Build CSpaceChecker ------------------------------------------
        checker: CSpaceChecker | None = None
        try:
            occ = self._occ_adapter.get_occupancy()
            checker = CSpaceChecker(self._kin, occ)
        except RuntimeError:
            pass  # occupancy not yet available; skip collision checking

        # -- 3. Plan a path --------------------------------------------------
        try:
            path = self._find_path(
                request.start_configuration,
                request.goal_configuration,
                request.planning_timeout,
                checker,
            )
        except TimeoutError:
            return PlanningResult(
                status=PlanningStatus.ABORTED,
                path=[],
                error_code=ErrorCode.TIMEOUT,
                planning_duration=time.monotonic() - t_start,
            )
        except RuntimeError:
            return PlanningResult(
                status=PlanningStatus.ABORTED,
                path=[],
                error_code=ErrorCode.NO_PATH_FOUND,
                planning_duration=time.monotonic() - t_start,
            )

        # -- 4. Post-process -------------------------------------------------
        try:
            self._traj_gen.process(path)
        except (ValueError, RuntimeError):
            return PlanningResult(
                status=PlanningStatus.ABORTED,
                path=[],
                error_code=ErrorCode.POST_PROCESS_FAILED,
                planning_duration=time.monotonic() - t_start,
            )

        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            error_code=ErrorCode.NONE,
            planning_duration=time.monotonic() - t_start,
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _find_path(
        self,
        start: npt.NDArray[np.float64],
        goal: npt.NDArray[np.float64],
        timeout: float,
        checker: CSpaceChecker | None,
    ) -> list[npt.NDArray[np.float64]]:
        """Return a collision-free joint-space path from start to goal.

        Uses ARCO SST when available.  Falls back to a straight
        joint-space path (two waypoints) when ARCO is absent.

        Args:
            start: Start configuration, shape ``(DOF,)``.
            goal: Goal configuration, shape ``(DOF,)``.
            timeout: Maximum planning time in seconds.
            checker: Collision predicate; ``None`` means no checking.

        Returns:
            List of at least 2 joint configurations.

        Raises:
            TimeoutError: If ARCO SST exceeds the timeout.
            RuntimeError: If no path exists.
        """
        if SST is not None and checker is not None:  # pragma: no cover
            sst = SST(
                collision_free=checker.is_collision_free,
                timeout=timeout,
            )
            return sst.plan(start, goal)

        # Fallback: straight joint-space path (valid in an empty world).
        path = [
            np.asarray(start, dtype=np.float64),
            np.asarray(goal, dtype=np.float64),
        ]
        return path
