"""Dubins race obstacle layout and occupancy builder (T11-02).

Loads the column forest from ``dubins_race_obstacles.yml`` and builds an
ARCO ``KDTreeOccupancy`` map for 2-D planning, mirroring
``arco.simulator.scenes.vehicle.VehicleScene._build_occupancy``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml
from arco.mapping import KDTreeOccupancy

_DEFAULT_OBSTACLE_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "worlds"
    / "dubins_race_obstacles.yml"
)


@dataclass(frozen=True)
class ColumnObstacle:
    """Circular column footprint in the warehouse floor plane."""

    x: float
    y: float
    radius: float
    height: float


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
    columns: tuple[ColumnObstacle, ...]


def default_obstacle_file() -> Path:
    """Return the bundled dubins race obstacle YAML path."""
    return _DEFAULT_OBSTACLE_FILE


def load_dubins_race_world(
    path: str | Path | None = None,
) -> DubinsRaceWorld:
    """Load column forest and planner parameters from YAML.

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

    columns: list[ColumnObstacle] = []
    for entry in data.get("columns", []):
        columns.append(
            ColumnObstacle(
                x=float(entry["x"]),
                y=float(entry["y"]),
                radius=float(entry.get("radius", 0.5)),
                height=float(entry.get("height", 2.5)),
            )
        )

    return DubinsRaceWorld(
        workspace_bounds=workspace_bounds,
        start_xy=start_xy,
        goal_xy=goal_xy,
        agent_lateral_offset=float(data.get("agent_lateral_offset", 0.6)),
        vehicle_radius=vehicle_radius,
        clearance_margin=clearance_margin,
        planner=dict(data.get("planner", {})),
        columns=tuple(columns),
    )


def column_centres(columns: tuple[ColumnObstacle, ...]) -> list[list[float]]:
    """Return 2-D obstacle centre points for KDTree occupancy."""
    return [[col.x, col.y] for col in columns]


def build_race_occupancy(
    world: DubinsRaceWorld,
) -> KDTreeOccupancy:
    """Build a shared KDTree occupancy map for both race agents.

    Args:
        world: Parsed dubins race world.

    Returns:
        ARCO occupancy with clearance = vehicle radius + margin.
    """
    clearance = world.vehicle_radius + world.clearance_margin
    return KDTreeOccupancy(column_centres(world.columns), clearance=clearance)


def load_workspace_bounds(
    path: str | Path | None = None,
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    """Return workspace bounds from the obstacle YAML."""
    return load_dubins_race_world(path).workspace_bounds
