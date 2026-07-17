"""Axis-aligned box obstacle used by occupancy helpers (shared)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned box obstacle in world frame [m]."""

    x_min: float
    y_min: float
    z_min: float
    x_max: float
    y_max: float
    z_max: float

    @property
    def centre(self) -> npt.NDArray[np.float64]:
        """Box centre position."""
        return np.array(
            [
                0.5 * (self.x_min + self.x_max),
                0.5 * (self.y_min + self.y_max),
                0.5 * (self.z_min + self.z_max),
            ],
            dtype=np.float64,
        )

    @property
    def half_extent(self) -> npt.NDArray[np.float64]:
        """Half-sizes along X, Y, Z."""
        return np.array(
            [
                0.5 * (self.x_max - self.x_min),
                0.5 * (self.y_max - self.y_min),
                0.5 * (self.z_max - self.z_min),
            ],
            dtype=np.float64,
        )
