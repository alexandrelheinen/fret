"""Dubins vehicle C-space collision checker (T11-01).

Tests whether a planar position ``(x, y)`` keeps the circular vehicle
footprint clear of rectangular structures loaded from
``dubins_race_obstacles.yml``.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt

from fret.planning.cspace_checker import occupancy_min_clearance
from fret.planning.dubins_obstacles import (
    DubinsRaceWorld,
    RectObstacle,
    circle_rect_clearance,
)


class DubinsCSpaceChecker:
    """Circular-footprint collision predicate for Dubins race planning.

    Args:
        occupancy: ARCO ``KDTreeOccupancy`` or duck-typed equivalent.
        vehicle_radius: Chassis radius used for footprint inflation [m].
        structures: Optional structure list for analytic fallback checks.
    """

    def __init__(
        self,
        occupancy: Any,
        *,
        vehicle_radius: float,
        structures: tuple[RectObstacle, ...] = (),
    ) -> None:
        self._occ = occupancy
        self._vehicle_radius = float(vehicle_radius)
        self._structures = structures

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
            structures=world.structures,
        )

    def is_collision_free(self, config: npt.NDArray[np.float64]) -> bool:
        """Return True when the vehicle footprint is clear at ``(x, y)``."""
        q = np.asarray(config, dtype=np.float64)
        x, y = float(q[0]), float(q[1])
        if self._structures:
            for rect in self._structures:
                if (
                    circle_rect_clearance(x, y, self._vehicle_radius, rect)
                    <= 0.0
                ):
                    return False
        pt = np.array([[x, y, 0.0]], dtype=np.float64)
        return occupancy_min_clearance(self._occ, pt) > 0.0

    def is_position_free(self, x: float, y: float) -> bool:
        """Return True when the planar position is collision-free."""
        return self.is_collision_free(np.array([x, y, 0.0], dtype=np.float64))
