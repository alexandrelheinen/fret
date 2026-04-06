"""KD-tree occupancy adapter for the ARCO-FRET integration.

Converts filtered obstacle point clouds (world frame) into a KD-tree–backed
occupancy representation compatible with ARCO planners.

**Occupancy semantics:**

- A query point is **occupied** when the nearest obstacle point lies within
  ``inflation_radius`` meters (inclusive).
- A query point is **free** when the nearest obstacle point is farther than
  ``inflation_radius`` meters.
- **Clearance** is ``max(0, nearest_distance − inflation_radius)``, measured in
  meters.  A clearance of ``0.0`` means the point is occupied.  A clearance of
  ``math.inf`` means the tree is empty (no obstacles known).

**Inflation policy:**

The ``inflation_radius`` adds a safety margin around every obstacle point so
that the robot body, which has non-zero volume, never contacts the raw cloud
surface.  A value of ``0.05 m`` (5 cm) is a reasonable default for tabletop
manipulation; larger robots or faster motions require a larger margin.

**Complexity:**

- Tree build: O(n log n) time, O(n) space (median-split KD-tree).
- Single occupancy query: O(log n) average case, O(n) worst case.
- Batch query of *m* points: O(m log n) average case.

**Rebuild strategy:**

Every call to :meth:`OccupancyAdapter.update` rebuilds the tree from scratch
(full rebuild).  This is safe and predictable for snapshot-based pipelines
running at ≤ 10 Hz with up to :attr:`OccupancyAdapter.max_points` obstacle
points.  The :attr:`OccupancyAdapter.is_stale` property can be used by callers
to detect when the tree has aged beyond :attr:`OccupancyAdapter.max_rebuild_age`
and trigger a fresh update.

No ROS 2 runtime dependency; the module can be imported and tested independently.

See also:
    ``docs/arco/issue-04-point-cloud-to-kdtree-occupancy-adapter.md`` —
    full specification and acceptance criteria.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Point = List[float]  # [x, y, z]


# ---------------------------------------------------------------------------
# Internal KD-tree implementation
# ---------------------------------------------------------------------------


class _KDNode:
    """A single node in a 3-D KD-tree.

    Attributes:
        point: The ``[x, y, z]`` obstacle point stored at this node.
        left: Left child subtree (points with axis-value < split value).
        right: Right child subtree (points with axis-value ≥ split value).
    """

    __slots__ = ("point", "left", "right")

    def __init__(
        self,
        point: Point,
        left: Optional[_KDNode],
        right: Optional[_KDNode],
    ) -> None:
        self.point = point
        self.left = left
        self.right = right


def _build(points: List[Point], depth: int = 0) -> Optional[_KDNode]:
    """Recursively build a balanced KD-tree by median splitting.

    Args:
        points: Mutable list of ``[x, y, z]`` triples.  The list is sorted
            in place during partitioning; callers must pass a copy if the
            original order must be preserved.
        depth: Current recursion depth; determines the split axis
            (``depth % 3``).

    Returns:
        Root node of the subtree, or ``None`` when ``points`` is empty.
    """
    if not points:
        return None

    axis = depth % 3
    points.sort(key=lambda p: p[axis])
    mid = len(points) // 2

    return _KDNode(
        points[mid],
        _build(points[:mid], depth + 1),
        _build(points[mid + 1 :], depth + 1),
    )


def _nearest_sq(
    node: Optional[_KDNode],
    target: Point,
    depth: int,
    best_sq: float,
) -> float:
    """Return the squared distance to the nearest point in the subtree.

    Uses the standard KD-tree branch-and-bound pruning: the far branch is
    only explored when the axis-aligned splitting plane is closer than the
    current best distance.

    Args:
        node: Current KD-tree node (``None`` terminates the recursion).
        target: Query point ``[x, y, z]``.
        depth: Current recursion depth (determines split axis).
        best_sq: Current best **squared** distance found so far.

    Returns:
        Updated best squared distance.
    """
    if node is None:
        return best_sq

    dx = node.point[0] - target[0]
    dy = node.point[1] - target[1]
    dz = node.point[2] - target[2]
    dist_sq = dx * dx + dy * dy + dz * dz

    if dist_sq < best_sq:
        best_sq = dist_sq

    axis = depth % 3
    diff = target[axis] - node.point[axis]
    near, far = (
        (node.left, node.right) if diff <= 0 else (node.right, node.left)
    )

    best_sq = _nearest_sq(near, target, depth + 1, best_sq)

    # Only explore the far branch when the axis-aligned plane is closer
    # than the current best distance.
    if diff * diff < best_sq:
        best_sq = _nearest_sq(far, target, depth + 1, best_sq)

    return best_sq


# ---------------------------------------------------------------------------
# Public adapter
# ---------------------------------------------------------------------------


class OccupancyAdapter:
    """KD-tree–backed occupancy adapter for ARCO collision queries.

    Builds an internal 3-D KD-tree from a filtered obstacle point cloud and
    answers :meth:`is_occupied`, :meth:`is_free`, and :meth:`clearance`
    queries against it.

    The tree is rebuilt on every call to :meth:`update` (full rebuild strategy).
    For snapshot-based pipelines running at ≤ 10 Hz this is both safe and
    efficient; the bounded :attr:`max_points` limit caps memory and rebuild
    time.

    Args:
        inflation_radius: Collision inflation radius in meters.  A query
            point is considered **occupied** when the nearest obstacle point
            is within this distance.  Must be strictly positive.
        max_points: Maximum number of obstacle points stored in the tree.
            Incoming clouds larger than this are truncated (first
            ``max_points`` entries are retained).  Caps peak memory use and
            rebuild time.
        max_rebuild_age: Maximum tree age in seconds before the adapter is
            considered stale.  Used by :attr:`is_stale`.

    Raises:
        ValueError: If ``inflation_radius``, ``max_points``, or
            ``max_rebuild_age`` violate their constraints.

    Example::

        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update(filtered_points)

        if adapter.is_occupied([0.5, 0.0, 0.8]):
            print("collision detected")

        print(f"clearance: {adapter.clearance([1.0, 0.0, 1.0]):.3f} m")
    """

    def __init__(
        self,
        inflation_radius: float = 0.05,
        max_points: int = 100_000,
        max_rebuild_age: float = 2.0,
    ) -> None:
        if inflation_radius <= 0.0:
            raise ValueError(
                f"inflation_radius must be positive, got {inflation_radius}"
            )
        if max_points <= 0:
            raise ValueError(f"max_points must be positive, got {max_points}")
        if max_rebuild_age <= 0.0:
            raise ValueError(
                f"max_rebuild_age must be positive, got {max_rebuild_age}"
            )

        self._inflation_radius = inflation_radius
        self._max_points = max_points
        self._max_rebuild_age = max_rebuild_age
        self._root: Optional[_KDNode] = None
        self._point_count: int = 0
        self._last_update_stamp: Optional[float] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def inflation_radius(self) -> float:
        """Collision inflation radius in meters."""
        return self._inflation_radius

    @property
    def max_points(self) -> int:
        """Maximum number of stored obstacle points."""
        return self._max_points

    @property
    def max_rebuild_age(self) -> float:
        """Maximum tree age in seconds before the adapter is considered stale."""
        return self._max_rebuild_age

    @property
    def point_count(self) -> int:
        """Number of obstacle points in the current KD-tree."""
        return self._point_count

    @property
    def last_update_stamp(self) -> Optional[float]:
        """POSIX timestamp of the last :meth:`update` call, or ``None``."""
        return self._last_update_stamp

    @property
    def is_stale(self) -> bool:
        """``True`` when no update has been received or the tree age exceeds
        :attr:`max_rebuild_age`."""
        if self._last_update_stamp is None:
            return True
        return (time.time() - self._last_update_stamp) > self._max_rebuild_age

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(self, points: List[Point]) -> None:
        """Rebuild the KD-tree from a new obstacle cloud snapshot.

        Incoming clouds larger than :attr:`max_points` are silently
        truncated (first ``max_points`` entries retained).  An empty cloud
        is valid; after the call :attr:`point_count` will be ``0`` and all
        subsequent queries will return ``is_free = True``.

        Build complexity: O(n log n) time, O(n) space.

        Args:
            points: Filtered obstacle points in the ``world`` frame, each a
                ``[x, y, z]`` triple in meters.
        """
        truncated = points[: self._max_points]
        # Copy each point so the internal tree is independent of the caller's list.
        self._root = _build([list(p) for p in truncated])
        self._point_count = len(truncated)
        self._last_update_stamp = time.time()

    # ------------------------------------------------------------------
    # Occupancy queries
    # ------------------------------------------------------------------

    def nearest_distance(self, point: Point) -> float:
        """Return the Euclidean distance to the nearest obstacle point.

        Query complexity: O(log n) average case, O(n) worst case.

        Args:
            point: Query point ``[x, y, z]`` in the ``world`` frame (meters).

        Returns:
            Distance in meters to the nearest obstacle, or ``math.inf`` when
            the tree is empty.
        """
        if self._root is None:
            return math.inf
        dist_sq = _nearest_sq(self._root, point, 0, math.inf)
        return math.sqrt(dist_sq)

    def is_occupied(self, point: Point) -> bool:
        """Return ``True`` when the query point is within the inflation radius.

        A point is **occupied** when the nearest obstacle is at most
        :attr:`inflation_radius` meters away (inclusive boundary).

        Query complexity: O(log n) average case, O(n) worst case.

        Args:
            point: Query point ``[x, y, z]`` in the ``world`` frame (meters).

        Returns:
            ``True`` if occupied, ``False`` if free.
        """
        return self.nearest_distance(point) <= self._inflation_radius

    def is_free(self, point: Point) -> bool:
        """Return ``True`` when the query point is outside the inflation radius.

        The logical complement of :meth:`is_occupied`.

        Query complexity: O(log n) average case, O(n) worst case.

        Args:
            point: Query point ``[x, y, z]`` in the ``world`` frame (meters).

        Returns:
            ``True`` if free, ``False`` if occupied.
        """
        return not self.is_occupied(point)

    def clearance(self, point: Point) -> float:
        """Return the clearance distance from the query point.

        Clearance is defined as::

            max(0.0, nearest_distance(point) − inflation_radius)

        A clearance of ``0.0`` means the point is occupied or on the
        inflation boundary.  A clearance of ``math.inf`` means the tree is
        empty (no obstacles are known).

        Query complexity: O(log n) average case, O(n) worst case.

        Args:
            point: Query point ``[x, y, z]`` in the ``world`` frame (meters).

        Returns:
            Clearance in meters (non-negative); ``math.inf`` when the tree is
            empty.
        """
        d = self.nearest_distance(point)
        if d == math.inf:
            return math.inf
        return max(0.0, d - self._inflation_radius)
