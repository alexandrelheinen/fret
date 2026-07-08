"""PPP warehouse obstacle layout (T10-08).

Loads static box obstacles ported from ARCO ``map/ppp.yml`` and converts
them to point-cloud samples for occupancy models used by ``PPPcSpaceChecker``.

Satisfies requirement FR-PLN-02 (PPP C-space planning domain).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt
import yaml

#: Default obstacle layout shipped with FRET (ARCO ``ppp.yml`` port).
_DEFAULT_OBSTACLE_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "worlds"
    / "ppp_warehouse_obstacles.yml"
)

#: MJCF 1:5 preview layout for MuJoCo / E2E validation (SC-v10).
_PREVIEW_OBSTACLE_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "worlds"
    / "ppp_warehouse_preview_obstacles.yml"
)

_PPP_JOINT_NAMES: list[str] = ["joint_x", "joint_y", "joint_z"]


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


def default_obstacle_file() -> Path:
    """Return the path to the bundled PPP warehouse obstacle YAML."""
    return _DEFAULT_OBSTACLE_FILE


def preview_obstacle_file() -> Path:
    """Return the path to the MJCF 1:5 preview obstacle YAML."""
    return _PREVIEW_OBSTACLE_FILE


def load_ppp_warehouse_obstacles(
    path: Path | None = None,
) -> list[BoxObstacle]:
    """Load PPP warehouse box obstacles from YAML.

    Args:
        path: YAML file path.  Defaults to ``ppp_warehouse_obstacles.yml``.

    Returns:
        List of ``BoxObstacle`` instances.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
        ValueError: If the file format is invalid.
    """
    obstacle_path = path if path is not None else default_obstacle_file()
    if not obstacle_path.is_file():
        raise FileNotFoundError(f"Obstacle file not found: {obstacle_path}")

    with obstacle_path.open() as fh:
        data = yaml.safe_load(fh)

    raw_boxes = data.get("obstacles", [])
    if not isinstance(raw_boxes, list):
        raise ValueError("'obstacles' must be a list of 6-element boxes")

    boxes: list[BoxObstacle] = []
    for idx, entry in enumerate(raw_boxes):
        if not isinstance(entry, list) or len(entry) != 6:
            raise ValueError(
                f"obstacles[{idx}] must be [x_min, y_min, z_min, x_max, y_max, z_max]"
            )
        x0, y0, z0, x1, y1, z1 = (float(v) for v in entry)
        boxes.append(
            BoxObstacle(
                x_min=x0,
                y_min=y0,
                z_min=z0,
                x_max=x1,
                y_max=y1,
                z_max=z1,
            )
        )
    return boxes


def load_ppp_warehouse_preview_obstacles(
    path: Path | None = None,
) -> list[BoxObstacle]:
    """Load PPP warehouse preview-scale obstacles from YAML.

    Args:
        path: YAML file path.  Defaults to ``ppp_warehouse_preview_obstacles.yml``.

    Returns:
        List of ``BoxObstacle`` instances for the MuJoCo preview scene.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
        ValueError: If the file format is invalid.
    """
    return load_ppp_warehouse_obstacles(
        path if path is not None else preview_obstacle_file()
    )


def load_preview_workspace_bounds(
    path: Path | None = None,
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    """Load preview workspace bounds from obstacle YAML.

    Returns:
        ``((x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi))`` in metres.
    """
    obstacle_path = path if path is not None else preview_obstacle_file()
    with obstacle_path.open() as fh:
        data = yaml.safe_load(fh)
    bounds = data.get("workspace_bounds", {})
    xb = bounds.get("x", [0.0, 12.0])
    yb = bounds.get("y", [0.0, 4.0])
    zb = bounds.get("z", [0.0, 3.0])
    return (
        (float(xb[0]), float(xb[1])),
        (float(yb[0]), float(yb[1])),
        (float(zb[0]), float(zb[1])),
    )


def boxes_to_point_cloud(
    boxes: list[BoxObstacle],
    *,
    samples_per_edge: int = 4,
) -> npt.NDArray[np.float64]:
    """Sample box surfaces as a point cloud for occupancy models.

    Args:
        boxes: Static warehouse obstacles.
        samples_per_edge: Samples along each box edge (including corners).

    Returns:
        Point cloud array, shape ``(N, 3)``.
    """
    if samples_per_edge < 2:
        raise ValueError("samples_per_edge must be at least 2")

    points: list[npt.NDArray[np.float64]] = []
    for box in boxes:
        hx, hy, hz = box.half_extent
        centre = box.centre
        for sx in np.linspace(-hx, hx, samples_per_edge):
            for sy in np.linspace(-hy, hy, samples_per_edge):
                for sz in np.linspace(-hz, hz, samples_per_edge):
                    # Keep surface voxels only (at least one coordinate on face).
                    on_face = (
                        abs(abs(sx) - hx) < 1e-9
                        or abs(abs(sy) - hy) < 1e-9
                        or abs(abs(sz) - hz) < 1e-9
                    )
                    if on_face:
                        points.append(centre + np.array([sx, sy, sz]))
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(points)


def is_ppp_kinematics(joint_names: list[str]) -> bool:
    """Return True when joint names identify the PPP gantry model."""
    return joint_names == _PPP_JOINT_NAMES


class BoxObstacleOccupancy:
    """Axis-aligned box occupancy for PPP warehouse obstacles.

    Computes signed clearance to box surfaces: positive outside, negative
    when a query point lies inside a box (expanded by ``contact_radius``).

    Compatible with ``occupancy_min_clearance`` and ``PPPcSpaceChecker``.

    Args:
        boxes: Static warehouse obstacles.
        contact_radius: Safety margin subtracted from clearance [m].
    """

    def __init__(
        self,
        boxes: list[BoxObstacle],
        *,
        contact_radius: float = 0.01,
    ) -> None:
        self._boxes = list(boxes)
        self._contact_radius = contact_radius

    def clearance(self, query_pts: npt.NDArray[np.float64]) -> float:
        """Return minimum clearance over all query points [m]."""
        pts = np.atleast_2d(query_pts)
        if pts.size == 0:
            return float("inf")
        return float(min(self._point_clearance(pt) for pt in pts))

    def _point_clearance(self, point: npt.NDArray[np.float64]) -> float:
        if not self._boxes:
            return float("inf")
        return float(
            min(self._point_box_clearance(point, box) for box in self._boxes)
        )

    def _point_box_clearance(
        self,
        point: npt.NDArray[np.float64],
        box: BoxObstacle,
    ) -> float:
        px, py, pz = float(point[0]), float(point[1]), float(point[2])
        margin = self._contact_radius
        inside = (
            box.x_min <= px <= box.x_max
            and box.y_min <= py <= box.y_max
            and box.z_min <= pz <= box.z_max
        )
        if inside:
            dist_faces = (
                px - box.x_min,
                box.x_max - px,
                py - box.y_min,
                box.y_max - py,
                pz - box.z_min,
                box.z_max - pz,
            )
            return -min(dist_faces) - margin

        cx = min(max(px, box.x_min), box.x_max)
        cy = min(max(py, box.y_min), box.y_max)
        cz = min(max(pz, box.z_min), box.z_max)
        return float(np.linalg.norm([px - cx, py - cy, pz - cz])) - margin


def build_box_obstacle_occupancy(
    boxes: list[BoxObstacle] | None = None,
    *,
    contact_radius: float = 0.01,
) -> BoxObstacleOccupancy:
    """Build box occupancy from the default warehouse or a custom list."""
    resolved = boxes if boxes is not None else load_ppp_warehouse_obstacles()
    return BoxObstacleOccupancy(resolved, contact_radius=contact_radius)
