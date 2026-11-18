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

    @property
    def size(self) -> npt.NDArray[np.float64]:
        """Full side lengths along X, Y, Z."""
        return 2.0 * self.half_extent

    def contains_point(
        self,
        point: npt.NDArray[np.float64],
        *,
        margin: float = 0.0,
    ) -> bool:
        """Return True when ``point`` lies inside the box expanded by ``margin``."""
        x_m, y_m, z_m = (
            float(v) for v in np.asarray(point, dtype=np.float64).reshape(3)
        )
        pad = float(margin)
        return (
            self.x_min - pad <= x_m <= self.x_max + pad
            and self.y_min - pad <= y_m <= self.y_max + pad
            and self.z_min - pad <= z_m <= self.z_max + pad
        )

    def sample_surface(
        self, density: float, *, rng: np.random.Generator | None = None
    ) -> npt.NDArray[np.float64]:
        """Sample points uniformly on all six faces (world frame).

        Args:
            density: Approximate samples per square metre.
            rng: Optional NumPy generator (deterministic tests / CI).

        Returns:
            Array of shape ``(N, 3)``.
        """
        gen = rng if rng is not None else np.random.default_rng()
        sx, sy, sz = (float(v) for v in self.size)
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        chunks: list[npt.NDArray[np.float64]] = []
        for sign in (-1.0, 1.0):
            n = max(1, int(round(density * sy * sz)))
            u = gen.uniform(-hy, hy, n)
            v = gen.uniform(-hz, hz, n)
            chunks.append(
                np.column_stack([np.full(n, sign * hx), u, v]).astype(
                    np.float64
                )
            )
        for sign in (-1.0, 1.0):
            n = max(1, int(round(density * sx * sz)))
            u = gen.uniform(-hx, hx, n)
            v = gen.uniform(-hz, hz, n)
            chunks.append(
                np.column_stack([u, np.full(n, sign * hy), v]).astype(
                    np.float64
                )
            )
        for sign in (-1.0, 1.0):
            n = max(1, int(round(density * sx * sy)))
            u = gen.uniform(-hx, hx, n)
            v = gen.uniform(-hy, hy, n)
            chunks.append(
                np.column_stack([u, v, np.full(n, sign * hz)]).astype(
                    np.float64
                )
            )
        local = np.vstack(chunks)
        return local + self.centre.reshape(1, 3)


def points_clear_of_boxes(
    points: npt.NDArray[np.float64],
    boxes: list[BoxObstacle],
    *,
    margin: float = 0.0,
) -> bool:
    """Return True when no point lies inside any box expanded by ``margin``."""
    cloud = np.asarray(points, dtype=np.float64)
    for point in cloud:
        for box in boxes:
            if box.contains_point(point, margin=margin):
                return False
    return True
