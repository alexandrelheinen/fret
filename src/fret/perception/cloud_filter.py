"""Point cloud filtering logic for the FRET perception pipeline.

Provides a pure-Python ``CloudFilter`` class that removes non-obstacle
points from a raw point cloud before the cloud is forwarded to the
occupancy adapter.  No ROS 2 runtime dependency.

Filtering stages (applied in this order by :meth:`CloudFilter.apply`):

1. **Floor filter** — discards points below ``floor_z + floor_margin``.
2. **Range filter** — discards points outside ``[min_range, max_range]``
   measured as the Euclidean distance from the origin.
3. **Self filter** — discards points inside any of the configured
   spherical exclusion zones (robot body geometry).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Point = List[float]  # [x, y, z]
ExclusionZone = Tuple[float, float, float, float]  # (cx, cy, cz, radius)


class CloudFilter:
    """Filter non-obstacle points from a raw point cloud.

    All filtering operations work on plain Python lists of ``[x, y, z]``
    triples (meters, ``world`` frame) and return new lists; the input is
    never modified.

    Args:
        floor_z: Z-coordinate of the floor plane (meters).
        floor_margin: Thickness of the floor exclusion band above
            ``floor_z`` (meters).  Points with ``z <= floor_z +
            floor_margin`` are removed.
        min_range: Minimum Euclidean distance from the sensor origin for
            a point to be retained (meters).
        max_range: Maximum Euclidean distance from the sensor origin for
            a point to be retained (meters).
        exclusion_zones: Spherical regions around robot links that are
            excluded from the output.  Each zone is a 4-tuple
            ``(cx, cy, cz, radius)`` in meters.

    Example::

        filt = CloudFilter(
            floor_z=0.0,
            floor_margin=0.02,
            min_range=0.1,
            max_range=5.0,
            exclusion_zones=[(0.0, 0.0, 0.5, 0.3)],
        )
        filtered = filt.apply(raw_points)
    """

    def __init__(
        self,
        floor_z: float = 0.0,
        floor_margin: float = 0.02,
        min_range: float = 0.1,
        max_range: float = 5.0,
        exclusion_zones: Optional[List[ExclusionZone]] = None,
    ) -> None:
        if floor_margin < 0.0:
            raise ValueError(
                f"floor_margin must be non-negative, got {floor_margin}"
            )
        if min_range < 0.0:
            raise ValueError(
                f"min_range must be non-negative, got {min_range}"
            )
        if max_range <= min_range:
            raise ValueError(
                f"max_range ({max_range}) must be greater than "
                f"min_range ({min_range})"
            )

        self._floor_z = floor_z
        self._floor_margin = floor_margin
        self._min_range = min_range
        self._max_range = max_range
        self._exclusion_zones: List[ExclusionZone] = (
            list(exclusion_zones) if exclusion_zones else []
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def floor_z(self) -> float:
        """Z-coordinate of the floor plane (meters)."""
        return self._floor_z

    @property
    def floor_margin(self) -> float:
        """Thickness of the floor exclusion band above ``floor_z`` (meters)."""
        return self._floor_margin

    @property
    def min_range(self) -> float:
        """Minimum retained Euclidean distance (meters)."""
        return self._min_range

    @property
    def max_range(self) -> float:
        """Maximum retained Euclidean distance (meters)."""
        return self._max_range

    @property
    def exclusion_zones(self) -> List[ExclusionZone]:
        """List of spherical self-exclusion zones ``(cx, cy, cz, radius)``."""
        return list(self._exclusion_zones)

    # ------------------------------------------------------------------
    # Individual filter stages
    # ------------------------------------------------------------------

    def filter_floor(self, points: List[Point]) -> List[Point]:
        """Remove floor and near-floor points.

        Retains only points with ``z > floor_z + floor_margin``.

        Args:
            points: Input list of ``[x, y, z]`` triples (meters).

        Returns:
            Filtered list containing only points above the floor band.
        """
        threshold = self._floor_z + self._floor_margin
        return [pt for pt in points if pt[2] > threshold]

    def filter_range(self, points: List[Point]) -> List[Point]:
        """Remove points outside the sensor range window.

        Retains only points with Euclidean distance ``d`` from the origin
        satisfying ``min_range <= d <= max_range``.

        Args:
            points: Input list of ``[x, y, z]`` triples (meters).

        Returns:
            Filtered list containing only in-range points.
        """
        result = []
        for pt in points:
            dist = math.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)
            if self._min_range <= dist <= self._max_range:
                result.append(pt)
        return result

    def filter_self(self, points: List[Point]) -> List[Point]:
        """Remove points inside robot self-exclusion zones.

        A point is removed when it falls inside *any* of the configured
        spherical exclusion zones.

        Args:
            points: Input list of ``[x, y, z]`` triples (meters).

        Returns:
            Filtered list with robot self-points removed.
        """
        if not self._exclusion_zones:
            return list(points)

        result = []
        for pt in points:
            inside = False
            for cx, cy, cz, radius in self._exclusion_zones:
                dx = pt[0] - cx
                dy = pt[1] - cy
                dz = pt[2] - cz
                if dx * dx + dy * dy + dz * dz <= radius * radius:
                    inside = True
                    break
            if not inside:
                result.append(pt)
        return result

    # ------------------------------------------------------------------
    # Composite apply
    # ------------------------------------------------------------------

    def apply(self, points: List[Point]) -> List[Point]:
        """Apply all filter stages in sequence and return clean points.

        The filtering order is:

        1. :meth:`filter_floor`
        2. :meth:`filter_range`
        3. :meth:`filter_self`

        Args:
            points: Raw input list of ``[x, y, z]`` triples (meters).

        Returns:
            Filtered list ready for the occupancy adapter.
        """
        filtered = self.filter_floor(points)
        filtered = self.filter_range(filtered)
        filtered = self.filter_self(filtered)
        return filtered
