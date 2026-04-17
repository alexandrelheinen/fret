"""Trajectory post-processing chain.

Applies three successive refinement stages to a raw ARCO planner path:

1. ``arco.planning.TrajectoryPruner`` — remove redundant waypoints.
2. ``arco.planning.TrajectoryOptimizer`` — time-optimal refinement with
   collision awareness and an IK hook.
3. ``arco.guidance.BSplineInterpolator`` — C² smooth interpolation.

The output is a ``trajectory_msgs/JointTrajectory`` ROS message ready for
the controller node.

Satisfies requirement FR-PLN-06.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

try:
    from arco.guidance import (
        BSplineInterpolator,  # type: ignore[import-untyped]
    )
    from arco.planning import (  # type: ignore[import-untyped]
        TrajectoryOptimizer,
        TrajectoryPruner,
    )
except ImportError:
    TrajectoryOptimizer = None  # type: ignore[assignment, misc]
    TrajectoryPruner = None  # type: ignore[assignment, misc]
    BSplineInterpolator = None  # type: ignore[assignment, misc]


class TrajectoryGenerator:
    """Run the ARCO post-processing chain on a raw joint-space path.

    Args:
        kinematics: Kinematics engine used by the optimizer as an IK callable
            and for feasibility checks.
    """

    def __init__(self, kinematics: Kinematics) -> None:
        raise NotImplementedError

    def process(self, path: list[npt.NDArray[np.float64]]) -> Any:
        """Apply the full post-processing chain to a raw planner path.

        Args:
            path: Raw waypoint sequence from ARCO; each element shape ``(DOF,)``.
                Must have at least 2 elements.

        Returns:
            A ``trajectory_msgs.msg.JointTrajectory`` ROS 2 message.

        Raises:
            ValueError: If ``path`` has fewer than 2 waypoints.
            RuntimeError: If any stage of the chain raises an unrecoverable
                error (mapped to ``ErrorCode.POST_PROCESS_FAILED`` by the
                caller).
        """
        raise NotImplementedError
