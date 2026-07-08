"""Dubins race obstacle layout and occupancy builder (T11-02).

Loads rectangular warehouse structures from ``dubins_race_obstacles.yml`` and
builds an ARCO ``KDTreeOccupancy`` map sampled from axis-aligned box surfaces.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml
from arco.mapping.occupancy import Occupancy

from fret.planning.ppp_obstacles import BoxObstacle
from fret.sitl_config import resolve_package_file

_DEFAULT_OBSTACLE_FILE = resolve_package_file(
    "config", "worlds", "dubins_race_obstacles.yml"
)


@dataclass(frozen=True)
class RectObstacle:
    """Axis-aligned rectangular post or wall segment on the warehouse floor."""

    x: float
    y: float
    hx: float
    hy: float
    height: float

    def to_box(self) -> BoxObstacle:
        """Return a 3-D box for occupancy sampling and analytic checks."""
        return BoxObstacle(
            x_min=self.x - self.hx,
            y_min=self.y - self.hy,
            z_min=0.0,
            x_max=self.x + self.hx,
            y_max=self.y + self.hy,
            z_max=self.height,
        )


# Backward-compatible alias used by early SC-v11 docs/tests.
ColumnObstacle = RectObstacle


@dataclass(frozen=True)
class DubinsRaceWorld:
    """Parsed SC-v11 world description."""

    workspace_bounds: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]
    start_xy: npt.NDArray[np.float64]
    goal_xy: npt.NDArray[np.float64]
    agent_lateral_offset: float
    vehicle_radius: float
    clearance_margin: float
    planner: dict[str, Any]
    structures: tuple[RectObstacle, ...]

    @property
    def columns(self) -> tuple[RectObstacle, ...]:
        """Legacy name for rectangular race structures."""
        return self.structures


def default_obstacle_file() -> Path:
    """Return the bundled dubins race obstacle YAML path."""
    return _DEFAULT_OBSTACLE_FILE


def _parse_rect(entry: dict[str, Any]) -> RectObstacle:
    hx = float(entry.get("hx", entry.get("half_x", 0.5)))
    hy = float(entry.get("hy", entry.get("half_y", 0.5)))
    if "radius" in entry and "hx" not in entry and "half_x" not in entry:
        legacy = float(entry["radius"])
        hx = hy = legacy
    return RectObstacle(
        x=float(entry["x"]),
        y=float(entry["y"]),
        hx=hx,
        hy=hy,
        height=float(entry.get("height", 2.5)),
    )


def load_dubins_race_world(
    path: str | Path | None = None,
) -> DubinsRaceWorld:
    """Load structure forest and planner parameters from YAML.

    Args:
        path: Optional override for ``dubins_race_obstacles.yml``.

    Returns:
        Parsed world description.

    Raises:
        FileNotFoundError: If the YAML file is missing.
        ValueError: If required keys are absent.
    """
    obstacle_path = Path(path) if path is not None else _DEFAULT_OBSTACLE_FILE
    if not obstacle_path.is_file():
        raise FileNotFoundError(f"Obstacle file not found: {obstacle_path}")

    with obstacle_path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid obstacle YAML: {obstacle_path}")

    bounds_raw = data.get("workspace_bounds", {})
    x_bounds = bounds_raw.get("x", [0.0, 24.0])
    y_bounds = bounds_raw.get("y", [0.0, 16.0])
    z_bounds = bounds_raw.get("z", [0.0, 4.0])
    workspace_bounds: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ] = (
        (float(x_bounds[0]), float(x_bounds[1])),
        (float(y_bounds[0]), float(y_bounds[1])),
        (float(z_bounds[0]), float(z_bounds[1])),
    )

    start_xy = np.asarray(data.get("start_xy", [2.0, 2.0]), dtype=np.float64)
    goal_xy = np.asarray(data.get("goal_xy", [22.0, 14.0]), dtype=np.float64)

    vehicle = data.get("vehicle", {})
    vehicle_radius = float(vehicle.get("radius", 0.42))
    clearance_margin = float(vehicle.get("clearance_margin", 0.28))

    structure_entries = data.get("structures")
    if structure_entries is None:
        structure_entries = data.get("columns", [])
    if not isinstance(structure_entries, list):
        structure_entries = []

    structures: list[RectObstacle] = []
    for entry in structure_entries:
        structures.append(_parse_rect(entry))

    for dead_end in data.get("dead_ends", []):
        for part in dead_end.get("parts", []):
            structures.append(_parse_rect(part))

    return DubinsRaceWorld(
        workspace_bounds=workspace_bounds,
        start_xy=start_xy,
        goal_xy=goal_xy,
        agent_lateral_offset=float(data.get("agent_lateral_offset", 0.6)),
        vehicle_radius=vehicle_radius,
        clearance_margin=clearance_margin,
        planner=dict(data.get("planner", {})),
        structures=tuple(structures),
    )


def structure_footprint_points(
    structures: tuple[RectObstacle, ...],
    *,
    samples_per_edge: int = 4,
) -> list[list[float]]:
    """Sample 2-D box perimeters for ``KDTreeOccupancy`` (planar planning)."""
    if samples_per_edge < 2:
        raise ValueError("samples_per_edge must be at least 2")

    points: list[list[float]] = []
    for struct in structures:
        box = struct.to_box()
        xs = np.linspace(box.x_min, box.x_max, samples_per_edge)
        ys = np.linspace(box.y_min, box.y_max, samples_per_edge)
        for x in xs:
            for y in ys:
                on_edge = (
                    abs(x - box.x_min) < 1e-9
                    or abs(x - box.x_max) < 1e-9
                    or abs(y - box.y_min) < 1e-9
                    or abs(y - box.y_max) < 1e-9
                )
                if on_edge:
                    points.append([float(x), float(y)])
    return points


def build_race_occupancy(
    world: DubinsRaceWorld,
) -> RectStructureOccupancy:
    """Build a shared analytic occupancy map for both race agents.

    Args:
        world: Parsed dubins race world.

    Returns:
        Rectangle occupancy with ``clearance = vehicle_radius + margin``.
    """
    if not world.structures:
        raise ValueError("Dubins race world produced an empty structure list")
    return RectStructureOccupancy(
        world.structures,
        vehicle_radius=world.vehicle_radius,
        clearance_margin=world.clearance_margin,
    )


def load_workspace_bounds(
    path: str | Path | None = None,
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    """Return workspace bounds from the obstacle YAML."""
    return load_dubins_race_world(path).workspace_bounds


def circle_rect_clearance(
    x: float,
    y: float,
    vehicle_radius: float,
    rect: RectObstacle,
) -> float:
    """Signed clearance between a circular vehicle footprint and a rectangle."""
    closest_x = min(max(x, rect.x - rect.hx), rect.x + rect.hx)
    closest_y = min(max(y, rect.y - rect.hy), rect.y + rect.hy)
    return float(np.hypot(x - closest_x, y - closest_y) - vehicle_radius)


def _point_rect_surface_distance(
    x: float,
    y: float,
    rect: RectObstacle,
) -> tuple[float, npt.NDArray[np.float64]]:
    """Return distance from ``(x, y)`` to the rectangle surface and closest point."""
    closest_x = min(max(x, rect.x - rect.hx), rect.x + rect.hx)
    closest_y = min(max(y, rect.y - rect.hy), rect.y + rect.hy)
    closest = np.array([closest_x, closest_y], dtype=np.float64)
    return float(np.linalg.norm([x - closest_x, y - closest_y])), closest


class RectStructureOccupancy(Occupancy):
    """Analytic axis-aligned box occupancy for Dubins race planning.

    Uses true rectangle geometry instead of sparse perimeter samples so
    clearance matches the vehicle footprint radius used in MJCF visuals.
    """

    def __init__(
        self,
        structures: tuple[RectObstacle, ...],
        *,
        vehicle_radius: float,
        clearance_margin: float,
    ) -> None:
        super().__init__()
        if vehicle_radius <= 0.0:
            raise ValueError("vehicle_radius must be positive")
        if clearance_margin < 0.0:
            raise ValueError("clearance_margin must be non-negative")
        self._structures = structures
        self._vehicle_radius = float(vehicle_radius)
        self._clearance_margin = float(clearance_margin)
        self.clearance = self._vehicle_radius + self._clearance_margin

    @property
    def vehicle_radius(self) -> float:
        """Planning footprint radius [m]."""
        return self._vehicle_radius

    @property
    def clearance_margin(self) -> float:
        """Extra surface clearance beyond the vehicle radius [m]."""
        return self._clearance_margin

    def nearest_obstacle(
        self, point: npt.NDArray[np.float64]
    ) -> tuple[float, npt.NDArray[np.float64]]:
        """Return distance to the nearest structure surface and that point."""
        x, y = float(point[0]), float(point[1])
        if not self._structures:
            return float("inf"), np.array([x, y], dtype=np.float64)
        best_dist = float("inf")
        best_pt = np.array([x, y], dtype=np.float64)
        for rect in self._structures:
            dist, closest = _point_rect_surface_distance(x, y, rect)
            if dist < best_dist:
                best_dist = dist
                best_pt = closest
        return best_dist, best_pt

    def is_occupied(self, point: npt.NDArray[np.float64]) -> bool:
        """Return True when the vehicle centre is closer than ``clearance``."""
        dist, _ = self.nearest_obstacle(point)
        return dist < self.clearance

    def query_distances(
        self, points: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return surface distances for a batch of planar query points."""
        pts = np.atleast_2d(np.asarray(points, dtype=np.float64))
        if pts.size == 0:
            return np.empty((0,), dtype=np.float64)
        if not self._structures:
            return np.full(pts.shape[0], np.inf, dtype=np.float64)

        x = pts[:, 0][:, np.newaxis]
        y = pts[:, 1][:, np.newaxis]
        best = np.full(pts.shape[0], np.inf, dtype=np.float64)
        for rect in self._structures:
            closest_x = np.clip(x, rect.x - rect.hx, rect.x + rect.hx)
            closest_y = np.clip(y, rect.y - rect.hy, rect.y + rect.hy)
            dist = np.hypot(x - closest_x, y - closest_y).ravel()
            best = np.minimum(best, dist)
        return best


def vehicle_body_clearance(
    x: float,
    y: float,
    theta: float,
    structures: tuple[RectObstacle, ...],
    *,
    vehicle_radius: float,
    half_length: float = 0.36,
    half_width: float = 0.22,
) -> float:
    """Minimum oriented-body clearance over centre and corner samples [m]."""
    offsets = [
        (0.0, 0.0),
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, half_width),
        (-half_length, -half_width),
    ]
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    clearances = []
    for ox, oy in offsets:
        px = x + cos_t * ox - sin_t * oy
        py = y + sin_t * ox + cos_t * oy
        sample_radius = vehicle_radius if ox == 0.0 and oy == 0.0 else 0.05
        if not structures:
            clearances.append(float("inf"))
            continue
        clearances.append(
            min(
                circle_rect_clearance(px, py, sample_radius, rect)
                for rect in structures
            )
        )
    return float(min(clearances))
