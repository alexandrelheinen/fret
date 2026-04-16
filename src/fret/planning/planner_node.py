"""ROS 2 Action server for C-space path planning via ARCO.

Implements the ``PlanRequest`` action server.  The node drives the full
planning pipeline:

1. Validate goal configuration against joint limits (→ ``ABORTED /
   INVALID_CONFIGURATION`` on failure).
2. Call ARCO SST planner with the current ``CSpaceChecker`` as collision
   predicate (→ ``ABORTED / TIMEOUT`` or ``NO_PATH_FOUND`` on failure).
3. Run the ``TrajectoryGenerator`` post-processing chain (→ ``ABORTED /
   POST_PROCESS_FAILED`` on exception).
4. Publish the resulting ``JointTrajectory`` and return ``SUCCEEDED``.

FSM states and transition table are defined in docs/interfaces.md.
Satisfies requirements FR-PLN-01 through FR-PLN-07.
"""

from __future__ import annotations

from fret.planning.cspace_checker import CSpaceChecker
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter


class PlannerNode:
    """ROS 2 Action server wrapping ARCO for C-space manipulator planning.

    This class is intended to be used as a ROS 2 node.  It holds a
    ``CSpaceChecker``, an ``OccupancyAdapter``, and a ``TrajectoryGenerator``
    and wires them together via the Action server callbacks.

    Args:
        model: Robot model name (e.g. ``"scara"``).  Used to instantiate the
            kinematics engine.
        occupancy_adapter: Pre-constructed adapter providing the live
            ``KDTreeOccupancy``.

    Note:
        The full ROS 2 node constructor (``super().__init__``) and the Action
        server setup are part of the Level 4 implementation.  This stub
        defines the public interface only.
    """

    def __init__(
        self,
        model: str,
        occupancy_adapter: OccupancyAdapter,
    ) -> None:
        raise NotImplementedError
