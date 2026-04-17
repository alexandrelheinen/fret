"""Occupancy adapter: OccupancyUpdatePayload → ARCO KDTreeOccupancy.

Bridges the FRET scene layer and the ARCO planning library.  Maintains a
live ``KDTreeOccupancy`` instance rebuilt from the latest obstacle payload,
ensuring ARCO never touches ROS topics directly.

Satisfies requirements FR-SCN-03, FR-SCN-04.
"""

from __future__ import annotations

from typing import Any

from fret.interfaces import OccupancyUpdatePayload

try:
    from arco.mapping import KDTreeOccupancy
except ImportError:
    KDTreeOccupancy = None


class OccupancyAdapter:
    """Wrap an ``OccupancyUpdatePayload`` into an ARCO ``KDTreeOccupancy``.

    The adapter holds a single live occupancy model that is atomically
    replaced on each ``update()`` call.  The planning layer calls
    ``get_occupancy()`` to obtain the current snapshot before invoking
    ARCO.
    """

    def __init__(self) -> None:
        raise NotImplementedError

    def update(self, payload: OccupancyUpdatePayload) -> None:
        """Rebuild the internal occupancy model from a new payload.

        Args:
            payload: Latest obstacle geometry in the ``world`` frame.
        """
        raise NotImplementedError

    def get_occupancy(self) -> Any:
        """Return the current ``arco.mapping.KDTreeOccupancy`` instance.

        Returns:
            A ready-to-query ``KDTreeOccupancy`` built from the last
            ``update()`` call.

        Raises:
            RuntimeError: If ``update()`` has not been called yet.
        """
        raise NotImplementedError
