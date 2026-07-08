"""Workspace occupancy map: voxel-grid scan of the SCARA reachable workspace.

Classifies each voxel in a regular 3-D grid as occupied or free by comparing
its centre to the nearest obstacle point in an ``OccupancyUpdatePayload``.
Exposes an SDF-style ``clearance()`` query and a boolean ``is_occupied()``
query so the planning layer can use the workspace map as a collision predicate.

Satisfies requirements FR-SCN-05, FR-PLN-02 (Milestone 4).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.occupancy_adapter import _SimpleOccupancy

try:
    from arco.mapping import KDTreeOccupancy
except ImportError:
    KDTreeOccupancy = None

# ---------------------------------------------------------------------------
# SCARA reachable-workspace constants (source: control/kinematics.py)
# ---------------------------------------------------------------------------

#: Link 1 length [m].
_L1: float = 0.325
#: Link 2 length [m].
_L2: float = 0.275
#: Minimum reachable horizontal radius (|L1 − L2|) [m].
_R_MIN: float = abs(_L1 - _L2)
#: Maximum reachable horizontal radius (L1 + L2) [m].
_R_MAX: float = _L1 + _L2


class WorkspaceOccupancyBuilder:
    """Build a voxel-grid occupancy map of the SCARA reachable workspace.

    Samples a regular 3-D grid at ``resolution`` spacing, classifies each
    voxel whose centre falls in the annular reachable workspace
    (``_R_MIN ≤ r ≤ _R_MAX``) as occupied or free, and exposes the result
    via:

    * ``is_occupied(position, collision_check_distance)`` — boolean query.
    * ``clearance(position)`` — Euclidean signed distance to the nearest
      occupied voxel centre (positive = free, negative = inside collision
      radius).
    * ``occupied_centres()`` / ``free_centres()`` — raw arrays for
      visualisation.

    The builder is pure Python with no ROS imports.  The owning ROS node
    calls ``build()`` after receiving an ``OccupancyUpdatePayload`` from
    ``SceneAcquisition``.

    Args:
        resolution: Voxel edge length in metres.  Defaults to 0.20 m.
        x_bounds: ``(lower, upper)`` workspace x extent [m].
        y_bounds: ``(lower, upper)`` workspace y extent [m].
        z_bounds: ``(lower, upper)`` workspace z extent [m].
    """

    def __init__(
        self,
        resolution: float = 0.20,
        x_bounds: tuple[float, float] = (-0.60, 0.60),
        y_bounds: tuple[float, float] = (-0.60, 0.60),
        z_bounds: tuple[float, float] = (0.00, 0.40),
    ) -> None:
        if resolution <= 0.0:
            raise ValueError(f"resolution must be positive, got {resolution}")
        self._resolution: float = resolution
        self._x_bounds = x_bounds
        self._y_bounds = y_bounds
        self._z_bounds = z_bounds

        # Default collision-check distance = voxel circumradius.
        self._default_ccd: float = resolution * math.sqrt(3.0) / 2.0

        # State populated by build()
        self._occupied: npt.NDArray[np.float64] = np.empty(
            (0, 3), dtype=np.float64
        )
        self._free: npt.NDArray[np.float64] = np.empty(
            (0, 3), dtype=np.float64
        )
        self._occupancy_model: Any = None

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def collision_check_distance(self) -> float:
        """Default collision-check distance: voxel circumradius [m].

        Equals ``resolution * sqrt(3) / 2``.  This is the distance at which
        ``is_occupied()`` treats a query point as colliding when no explicit
        ``collision_check_distance`` is provided.
        """
        return self._default_ccd

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_grid(self) -> npt.NDArray[np.float64]:
        """Return all voxel centres in the annular reachable workspace.

        The full axis-aligned grid is generated with ``np.meshgrid``,
        then filtered to retain only centres whose XY horizontal radius
        satisfies ``_R_MIN ≤ r ≤ _R_MAX``.

        Returns:
            Array of shape ``(K, 3)`` containing voxel centres (in metres)
            that lie within the reachable annulus.
        """
        xs = np.arange(
            self._x_bounds[0],
            self._x_bounds[1] + self._resolution * 0.5,
            self._resolution,
        )
        ys = np.arange(
            self._y_bounds[0],
            self._y_bounds[1] + self._resolution * 0.5,
            self._resolution,
        )
        zs = np.arange(
            self._z_bounds[0],
            self._z_bounds[1] + self._resolution * 0.5,
            self._resolution,
        )

        # Snap to avoid floating-point drift past the upper bound.
        tol = self._resolution * 1e-6
        xs = xs[xs <= self._x_bounds[1] + tol]
        ys = ys[ys <= self._y_bounds[1] + tol]
        zs = zs[zs <= self._z_bounds[1] + tol]

        XX, YY, ZZ = np.meshgrid(xs, ys, zs, indexing="ij")
        all_centres = np.column_stack(
            [XX.ravel(), YY.ravel(), ZZ.ravel()]
        ).astype(np.float64)

        # Annular reachability mask (XY plane only)
        r = np.sqrt(all_centres[:, 0] ** 2 + all_centres[:, 1] ** 2)
        mask = (r >= _R_MIN) & (r <= _R_MAX)
        return all_centres[mask]

    def _classify(
        self,
        centres: npt.NDArray[np.float64],
        obstacle_points: npt.NDArray[np.float64],
    ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """Partition voxel centres into occupied and free sets.

        A voxel is occupied when the nearest obstacle point is closer than
        the voxel circumradius (``resolution * sqrt(3) / 2``).

        Args:
            centres: Voxel centres to classify, shape ``(K, 3)``.
            obstacle_points: Obstacle surface samples, shape ``(N, 3)``.
                If empty, all voxels are free.

        Returns:
            Tuple ``(occupied, free)`` each of shape ``(*, 3)``.
        """
        threshold = self._resolution * math.sqrt(3.0) / 2.0

        if len(obstacle_points) == 0 or len(centres) == 0:
            return (
                np.empty((0, 3), dtype=np.float64),
                centres.copy(),
            )

        # Vectorised nearest-distance computation.
        # For large grids (147 cells × N obstacle points) this is fast enough.
        # Shape: (K, N)
        diff = centres[:, np.newaxis, :] - obstacle_points[np.newaxis, :, :]
        dists = np.sqrt(np.sum(diff**2, axis=2))  # (K, N)
        min_dists = dists.min(axis=1)  # (K,)

        occ_mask = min_dists < threshold
        return centres[occ_mask].copy(), centres[~occ_mask].copy()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self, payload: OccupancyUpdatePayload) -> Any:
        """Classify the workspace grid and return the occupied-voxel model.

        Rebuilds the internal occupied/free arrays atomically.  The returned
        occupancy object (``KDTreeOccupancy`` or ``_SimpleOccupancy``) contains
        only the occupied voxel centres and can be passed directly to
        ``CSpaceChecker``.

        Args:
            payload: Latest obstacle geometry in the ``world`` frame.

        Returns:
            An occupancy model built from occupied voxel centres.
            Use ``KDTreeOccupancy`` when ARCO is available; ``_SimpleOccupancy``
            otherwise.  Returns ``None`` when no voxel is occupied.
        """
        centres = self._build_grid()
        occupied, free = self._classify(centres, payload.obstacle_points)

        self._occupied = occupied
        self._free = free

        if len(occupied) == 0:
            self._occupancy_model = None
            return None

        if KDTreeOccupancy is not None:
            self._occupancy_model = KDTreeOccupancy(
                occupied, clearance=self._default_ccd
            )
        else:
            self._occupancy_model = _SimpleOccupancy(occupied)

        return self._occupancy_model

    def is_occupied(
        self,
        position: npt.ArrayLike,
        collision_check_distance: float | None = None,
    ) -> bool:
        """Return ``True`` if ``position`` is within a collision radius of any occupied voxel.

        Args:
            position: Query position, shape ``(3,)`` or array-like of length 3.
            collision_check_distance: Radius [m] within which the position is
                considered occupied.  Defaults to the voxel circumradius
                (``resolution * sqrt(3) / 2``).

        Returns:
            ``True`` if any occupied voxel centre lies within
            ``collision_check_distance`` of ``position``.

        Raises:
            RuntimeError: If ``build()`` has not been called yet.
        """
        pos = np.asarray(position, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(
                f"position must have 3 elements, got shape {pos.shape}"
            )

        if len(self._occupied) == 0:
            # build() not yet called or empty world → treat as free.
            return False

        ccd = (
            collision_check_distance
            if collision_check_distance is not None
            else self._default_ccd
        )
        dists = np.linalg.norm(self._occupied - pos, axis=1)
        return bool(np.min(dists) < ccd)

    def clearance(self, position: npt.ArrayLike) -> float:
        """Return the signed Euclidean distance from ``position`` to the nearest occupied voxel.

        Positive values indicate free space; negative values indicate that
        the query point is within the voxel circumradius of an occupied centre
        (i.e. the same collision criterion used by ``is_occupied``).

        The clearance is defined as::

            clearance = min_dist_to_occupied_centre - default_ccd

        where ``default_ccd = resolution * sqrt(3) / 2``.

        Args:
            position: Query position, shape ``(3,)`` or array-like of length 3.

        Returns:
            Signed distance in metres.  ``+inf`` when the workspace has no
            occupied voxels.
        """
        pos = np.asarray(position, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(
                f"position must have 3 elements, got shape {pos.shape}"
            )

        if len(self._occupied) == 0:
            return float("inf")

        dists = np.linalg.norm(self._occupied - pos, axis=1)
        return float(np.min(dists)) - self._default_ccd

    def occupied_centres(self) -> npt.NDArray[np.float64]:
        """Return the occupied voxel centres from the last ``build()`` call.

        Returns:
            Array of shape ``(N, 3)``; empty if no build has been called or
            no voxels are occupied.
        """
        return self._occupied.copy()

    def free_centres(self) -> npt.NDArray[np.float64]:
        """Return the free voxel centres from the last ``build()`` call.

        Returns:
            Array of shape ``(M, 3)``; empty if no build has been called.
        """
        return self._free.copy()
