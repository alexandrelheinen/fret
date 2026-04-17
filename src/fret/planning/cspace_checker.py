"""C-space collision checker: FK + KDTreeOccupancy.

Bridges FRET's kinematics engine and ARCO's occupancy model to provide a
joint-space collision predicate.  For each query configuration ``q``, FK
maps ``q`` to world-frame link positions which are then tested against the
``KDTreeOccupancy``.

Satisfies requirement FR-PLN-02.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

try:
    from arco.mapping import KDTreeOccupancy
except ImportError:
    KDTreeOccupancy = None


class CSpaceChecker:
    """Query collision-free status for a joint configuration via FK + occupancy.

    Args:
        kinematics: Kinematics engine for the active robot model.  Used to
            evaluate ``FK(q)`` for each link when checking clearance.
        occupancy: An ``arco.mapping.KDTreeOccupancy`` instance provided by
            ``scene.OccupancyAdapter``.
    """

    def __init__(self, kinematics: Kinematics, occupancy: Any) -> None:
        raise NotImplementedError

    def is_collision_free(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        """Return True if the configuration does not collide with any obstacle.

        Args:
            configuration: Joint configuration, shape ``(DOF,)``.

        Returns:
            ``True`` if the minimum link-to-obstacle clearance is positive.
        """
        raise NotImplementedError

    def clearance(self, configuration: npt.NDArray[np.float64]) -> float:
        """Return the minimum world-frame clearance from any obstacle.

        Args:
            configuration: Joint configuration, shape ``(DOF,)``.

        Returns:
            Minimum clearance in meters.  Negative values indicate
            penetration.
        """
        raise NotImplementedError
