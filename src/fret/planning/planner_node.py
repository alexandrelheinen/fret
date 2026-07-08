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

import pathlib
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
from fret.planning.cspace_checker import (
    CollisionBackend,
    CSpaceChecker,
    make_cspace_checker,
)
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter

try:
    from arco.planning import SSTPlanner
except ImportError:
    SSTPlanner = None


class _CSpaceOccupancy:
    """Adapts ``CSpaceChecker`` to the ARCO ``Occupancy`` interface.

    ``SSTPlanner`` operates in joint space and calls ``is_occupied(q)`` where
    ``q`` is a joint configuration.  This adapter forwards the call to
    ``CSpaceChecker.is_collision_free``, which performs FK + world-frame
    obstacle lookup internally.

    Args:
        checker: Configured ``CSpaceChecker`` instance.
    """

    def __init__(self, checker: "CSpaceChecker") -> None:
        self._checker = checker

    def is_occupied(self, point: npt.NDArray[np.float64]) -> bool:
        """Return True if the joint configuration collides with an obstacle."""
        return not self._checker.is_collision_free(point)


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
        collision_backend: PPP collision backend (``analytic`` or ``mujoco``).
        scenario: Scenario stem for MJCF resolution (MuJoCo backend).
        mjcf_path: Optional MJCF override for MuJoCo collision checks.
    """

    def __init__(
        self,
        model: str,
        occupancy_adapter: OccupancyAdapter,
        *,
        occupancy: Any | None = None,
        collision_backend: CollisionBackend = "analytic",
        scenario: str = "ppp_warehouse",
        mjcf_path: str | pathlib.Path | None = None,
        workspace_bounds: (
            tuple[
                tuple[float, float],
                tuple[float, float],
                tuple[float, float],
            ]
            | None
        ) = None,
    ) -> None:
        from fret.control.kinematics import Kinematics

        self._kin = Kinematics(model)
        self._occ_adapter = occupancy_adapter
        self._occ_direct = occupancy
        self._traj_gen = TrajectoryGenerator(self._kin)
        self._collision_backend = collision_backend
        self._scenario = scenario
        self._mjcf_path = mjcf_path
        self._workspace_bounds = workspace_bounds

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
        checker: CSpaceChecker | Any | None = None
        try:
            occ = (
                self._occ_direct
                if self._occ_direct is not None
                else self._occ_adapter.get_occupancy()
            )
            checker = make_cspace_checker(
                self._kin,
                occ,
                collision_backend=self._collision_backend,
                scenario=self._scenario,
                mjcf_path=self._mjcf_path,
                workspace_bounds=self._workspace_bounds,
            )
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
            if checker is not None:
                self._traj_gen.set_collision_context(
                    _CSpaceOccupancy(checker),
                    np.full(self._kin.dof, 0.1, dtype=np.float64),
                )
            else:
                self._traj_gen.clear_collision_context()
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
        if SSTPlanner is not None and checker is not None:  # pragma: no cover
            if self._workspace_bounds is not None:
                (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi) = (
                    self._workspace_bounds
                )
                bounds = [
                    (x_lo, x_hi),
                    (y_lo, y_hi),
                    (z_lo, z_hi),
                ]
            else:
                limits = self._kin.joint_limits  # shape (DOF, 2)
                bounds = [(float(lo), float(hi)) for lo, hi in limits]
            sst = SSTPlanner(
                occupancy=_CSpaceOccupancy(checker),
                bounds=bounds,
                max_sample_count=8000,
                step_size=0.25,
                goal_tolerance=0.1,
                witness_radius=0.15,
                goal_bias=0.15,
            )
            result: list[npt.NDArray[np.float64]] | None = sst.plan(
                np.asarray(start, dtype=np.float64),
                np.asarray(goal, dtype=np.float64),
            )
            if result is None:
                raise RuntimeError("SSTPlanner found no collision-free path")
            return result

        # Fallback: straight joint-space path (valid in an empty world).
        path = [
            np.asarray(start, dtype=np.float64),
            np.asarray(goal, dtype=np.float64),
        ]
        return path
