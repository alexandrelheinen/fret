"""Typed data structures for all FRET inter-module boundaries.

Every dataclass in this module corresponds to a contract defined in
docs/interfaces.md.  Invariants are documented inline and enforced by
``__post_init__`` where practical.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

from fret.interfaces.enums import ErrorCode, PlanningStatus


@dataclass
class OccupancyUpdatePayload:
    """Obstacle geometry payload crossing the Scene → Planning boundary.

    Constructed by ``scene.OccupancyAdapter``; consumed by
    ``planning.CSpaceChecker``.  All points must be expressed in the
    ``world`` coordinate frame before construction.

    Attributes:
        obstacle_points: Array of obstacle surface samples, shape ``(N, 3)``,
            dtype ``float64``, in the ``world`` frame.  ``N == 0`` is valid
            (empty scene).
        timestamp: POSIX timestamp of the originating PointCloud2 message.
        frame_id: Coordinate frame of ``obstacle_points``.  Must be ``"world"``.
    """

    obstacle_points: npt.NDArray[np.float64]
    timestamp: float
    frame_id: str

    def __post_init__(self) -> None:
        if self.frame_id != "world":
            raise ValueError(
                f"OccupancyUpdatePayload frame_id must be 'world', "
                f"got '{self.frame_id}'"
            )
        if (
            self.obstacle_points.ndim != 2
            or self.obstacle_points.shape[1] != 3
        ):
            raise ValueError(
                f"obstacle_points must have shape (N, 3), "
                f"got {self.obstacle_points.shape}"
            )


@dataclass
class RobotState:
    """Snapshot of joint state at a given instant.

    Produced by ``control.StateEstimator``; consumed by
    ``planning.CSpaceChecker`` at planning-request time.

    Attributes:
        joint_positions: Joint positions in radians (revolute) or meters
            (prismatic), shape ``(DOF,)``.
        joint_velocities: Joint velocities in rad/s or m/s, shape ``(DOF,)``.
        joint_names: URDF joint names in the same order as the position /
            velocity arrays, length ``DOF``.
        timestamp: POSIX timestamp of the originating JointState message.
    """

    joint_positions: npt.NDArray[np.float64]
    joint_velocities: npt.NDArray[np.float64]
    joint_names: list[str]
    timestamp: float

    def __post_init__(self) -> None:
        dof = len(self.joint_names)
        if self.joint_positions.shape != (dof,):
            raise ValueError(
                f"joint_positions shape {self.joint_positions.shape} "
                f"does not match joint_count {dof}"
            )
        if self.joint_velocities.shape != (dof,):
            raise ValueError(
                f"joint_velocities shape {self.joint_velocities.shape} "
                f"does not match joint_count {dof}"
            )


@dataclass
class PlanningRequest:
    """Input to the Planning layer.

    Maps one-to-one to the Action goal fields of ``PlanRequest.action``.

    Attributes:
        start_configuration: Start joint configuration, shape ``(DOF,)``.
        goal_configuration: Goal joint configuration, shape ``(DOF,)``.
        planning_timeout: Maximum planning time in seconds; must be > 0.
        scenario_id: Human-readable identifier used in logs and feedback.
    """

    start_configuration: npt.NDArray[np.float64]
    goal_configuration: npt.NDArray[np.float64]
    planning_timeout: float
    scenario_id: str

    def __post_init__(self) -> None:
        if self.planning_timeout <= 0.0:
            raise ValueError(
                f"planning_timeout must be > 0, got {self.planning_timeout}"
            )
        if self.start_configuration.shape != self.goal_configuration.shape:
            raise ValueError(
                "start_configuration and goal_configuration must have the "
                f"same shape, got {self.start_configuration.shape} vs "
                f"{self.goal_configuration.shape}"
            )


@dataclass
class PlanningResult:
    """Output of the Planning layer.

    Maps one-to-one to the Action result fields of ``PlanRequest.action``.

    Attributes:
        status: High-level outcome of the planning request.
        path: Sequence of joint configurations; each element shape ``(DOF,)``.
            Empty list when ``status != SUCCESS``.
        error_code: Structured error code; ``NONE`` when ``status == SUCCESS``.
        planning_duration: Wall-clock planning time in seconds.
        iteration_count: Number of planner iterations executed.
    """

    status: PlanningStatus
    path: list[npt.NDArray[np.float64]] = field(default_factory=list)
    error_code: ErrorCode = ErrorCode.NONE
    planning_duration: float = 0.0
    iteration_count: int = 0

    def __post_init__(self) -> None:
        if self.status == PlanningStatus.SUCCESS and len(self.path) < 2:
            raise ValueError(
                "PlanningResult with SUCCESS must have at least 2 waypoints"
            )
        if self.status != PlanningStatus.SUCCESS and len(self.path) != 0:
            raise ValueError(
                "PlanningResult with non-SUCCESS status must have empty path"
            )
        if (
            self.status == PlanningStatus.SUCCESS
            and self.error_code != ErrorCode.NONE
        ):
            raise ValueError(
                "PlanningResult with SUCCESS must have error_code NONE"
            )
