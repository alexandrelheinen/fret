"""Occupancy adapter: OccupancyUpdatePayload → ARCO KDTreeOccupancy.

Bridges the FRET scene layer and the ARCO planning library.  Maintains a
live ``KDTreeOccupancy`` instance rebuilt from the latest obstacle payload,
ensuring ARCO never touches ROS topics directly.

When ARCO is not installed, a pure-Python fallback occupancy model is used
that answers ``clearance()`` queries using a brute-force nearest-neighbour
search.

Satisfies requirements FR-SCN-03, FR-SCN-04.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt

from fret.interfaces import OccupancyUpdatePayload

try:
    from arco.mapping import KDTreeOccupancy
except ImportError:
    KDTreeOccupancy = None


class _SimpleOccupancy:
    """Pure-Python point-cloud occupancy model used when ARCO is absent.

    Computes clearance as the minimum Euclidean distance from any query
    position to any obstacle point, minus a contact-radius tolerance.

    Args:
        points: Obstacle surface samples, shape ``(N, 3)``.
        contact_radius: Distance at which clearance becomes zero [m].
    """

    _CONTACT_RADIUS: float = 0.01  # 1 cm

    def __init__(self, points: npt.NDArray[np.float64]) -> None:
        self._points = points.copy()

    def clearance(self, query_pts: npt.NDArray[np.float64]) -> float:
        """Return the minimum clearance from any query point to the cloud.

        Args:
            query_pts: One or more query positions, shape ``(3,)`` or
                ``(N, 3)``.

        Returns:
            Minimum clearance in metres; positive = free, negative = inside.
        """
        query_pts = np.atleast_2d(query_pts)
        if len(self._points) == 0:
            return float("inf")
        min_dist = float("inf")
        for pt in query_pts:
            dists = np.linalg.norm(self._points - pt, axis=1)
            min_dist = min(min_dist, float(np.min(dists)))
        return min_dist - self._CONTACT_RADIUS


class OccupancyAdapter:
    """Wrap an ``OccupancyUpdatePayload`` into an occupancy model.

    Uses ARCO ``KDTreeOccupancy`` when available; falls back to
    ``_SimpleOccupancy`` otherwise.  The adapter holds a single live
    occupancy model that is atomically replaced on each ``update()`` call.
    The planning layer calls ``get_occupancy()`` to obtain the current
    snapshot before invoking the planner.
    """

    def __init__(self) -> None:
        self._occupancy: Any = None

    def update(self, payload: OccupancyUpdatePayload) -> None:
        """Rebuild the internal occupancy model from a new payload.

        Args:
            payload: Latest obstacle geometry in the ``world`` frame.
        """
        # ARCO's KDTreeOccupancy requires at least one obstacle point, so an
        # empty payload (obstacle-free scene) falls back to _SimpleOccupancy,
        # which reports infinite clearance everywhere.
        if KDTreeOccupancy is not None and len(payload.obstacle_points) > 0:
            # Use a 5 cm clearance radius — matches the 6 cm column width
            # with a small buffer while keeping most of the workspace reachable.
            self._occupancy = KDTreeOccupancy(
                payload.obstacle_points, clearance=0.05
            )
        else:
            self._occupancy = _SimpleOccupancy(payload.obstacle_points)

    def get_occupancy(self) -> Any:
        """Return the current occupancy model.

        Returns:
            A ready-to-query occupancy object (``KDTreeOccupancy`` or
            ``_SimpleOccupancy``) built from the last ``update()`` call.

        Raises:
            RuntimeError: If ``update()`` has not been called yet.
        """
        if self._occupancy is None:
            raise RuntimeError(
                "OccupancyAdapter.get_occupancy() called before update(). "
                "Call update() with an OccupancyUpdatePayload first."
            )
        return self._occupancy
