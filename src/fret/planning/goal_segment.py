"""Task-space goal region shaped as a straight line segment.

Planners that accept a :class:`GoalSegment` treat every point of the segment
as a valid goal. A pick-and-place drop uses a vertical segment through the
basket centre, starting at a minimum clearance above the rim and running
upward: the arm may release at any height on that segment.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class GoalSegment:
    """Straight segment in the world frame, in metres.

    ``start`` and ``end`` are the endpoints. The segment has positive length.
    """

    start: npt.NDArray[np.float64]
    end: npt.NDArray[np.float64]

    def __post_init__(self) -> None:
        start = np.asarray(self.start, dtype=np.float64).reshape(3).copy()
        end = np.asarray(self.end, dtype=np.float64).reshape(3).copy()
        if float(np.linalg.norm(end - start)) <= 1e-9:
            raise ValueError("GoalSegment length must be positive")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)

    @classmethod
    def vertical(
        cls,
        xy: npt.NDArray[np.float64],
        z_min: float,
        z_max: float,
    ) -> GoalSegment:
        """Vertical segment at ``xy`` from ``z_min`` up to ``z_max`` [m]."""
        x_m, y_m = (
            float(v) for v in np.asarray(xy, dtype=np.float64).reshape(2)
        )
        z0 = float(z_min)
        z1 = float(z_max)
        if z1 <= z0:
            raise ValueError(
                f"vertical GoalSegment needs z_max > z_min, got {z0} .. {z1}"
            )
        return cls(
            start=np.array([x_m, y_m, z0], dtype=np.float64),
            end=np.array([x_m, y_m, z1], dtype=np.float64),
        )

    @property
    def length(self) -> float:
        """Euclidean length [m]."""
        return float(np.linalg.norm(self.end - self.start))

    def point_at(self, t: float) -> npt.NDArray[np.float64]:
        """Point at fraction ``t`` in ``[0, 1]`` from ``start`` toward ``end``."""
        fraction = float(np.clip(t, 0.0, 1.0))
        return self.start + fraction * (self.end - self.start)

    def distance(self, point: npt.NDArray[np.float64]) -> float:
        """Shortest distance [m] from ``point`` to this segment."""
        query = np.asarray(point, dtype=np.float64).reshape(3)
        direction = self.end - self.start
        length_sq = float(direction @ direction)
        fraction = float(
            np.clip((query - self.start) @ direction / length_sq, 0.0, 1.0)
        )
        closest = self.start + fraction * direction
        return float(np.linalg.norm(query - closest))


def sample_polyline(
    points: npt.NDArray[np.float64],
    *,
    per_edge: int = 4,
) -> npt.NDArray[np.float64]:
    """Sample a polyline, including the last vertex.

    Args:
        points: Vertices, shape ``(N, 3)``, ``N >= 2``.
        per_edge: Samples on each edge, not counting the edge's end vertex.
    """
    vertices = np.asarray(points, dtype=np.float64)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or vertices.shape[0] < 2:
        raise ValueError(
            f"polyline vertices must have shape (N, 3), got {vertices.shape}"
        )
    count = max(1, int(per_edge))
    samples: list[npt.NDArray[np.float64]] = []
    for start, end in zip(vertices[:-1], vertices[1:]):
        for fraction in np.linspace(0.0, 1.0, count, endpoint=False):
            samples.append((1.0 - fraction) * start + fraction * end)
    samples.append(vertices[-1].copy())
    return np.stack(samples, axis=0)
