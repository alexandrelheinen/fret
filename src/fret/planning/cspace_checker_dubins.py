"""Dubins vehicle C-space collision checker (T11-01).

Tests whether a planar position ``(x, y)`` keeps the circular vehicle
footprint clear of column obstacles loaded from ``dubins_race_obstacles.yml``.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt

from fret.planning.cspace_checker import occupancy_min_clearance
from fret.planning.dubins_obstacles import ColumnObstacle, DubinsRaceWorld


class DubinsCSpaceChecker:
    """Circular-footprint collision predicate for Dubins race planning.

    Args:
        occupancy: ARCO ``KDTreeOccupancy`` or duck-typed equivalent.
        vehicle_radius: Chassis radius used for footprint inflation [m].
        columns: Optional column list for analytic fallback checks.
    """

    def __init__(
        self,
        occupancy: Any,
        *,
        vehicle_radius: float,
        columns: tuple[ColumnObstacle, ...] = (),
    ) -> None:
        self._occ = occupancy
        self._vehicle_radius = float(vehicle_radius)
        self._columns = columns

    @classmethod
    def from_world(
        cls,
        world: DubinsRaceWorld,
        occupancy: Any,
    ) -> DubinsCSpaceChecker:
        """Build a checker using vehicle radius from the world YAML."""
        return cls(
            occupancy,
            vehicle_radius=world.vehicle_radius,
            columns=world.columns,
        )

    def is_collision_free(self, config: npt.NDArray[np.float64]) -> bool:
        """Return True when the vehicle footprint is clear at ``(x, y)``."""
        q = np.asarray(config, dtype=np.float64)
        x, y = float(q[0]), float(q[1])
        if self._columns:
            for col in self._columns:
                dist = float(np.hypot(x - col.x, y - col.y))
                if dist < self._vehicle_radius + col.radius:
                    return False
        pt = np.array([[x, y, 0.0]], dtype=np.float64)
        return occupancy_min_clearance(self._occ, pt) > 0.0

    def is_position_free(self, x: float, y: float) -> bool:
        """Return True when the planar position is collision-free."""
        return self.is_collision_free(np.array([x, y, 0.0], dtype=np.float64))
