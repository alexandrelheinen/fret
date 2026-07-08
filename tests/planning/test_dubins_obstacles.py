"""Unit tests for Dubins race obstacle loading (T11-02)."""

from __future__ import annotations

from fret.planning.dubins_obstacles import (
    build_race_occupancy,
    default_obstacle_file,
    load_dubins_race_world,
)


def test_default_obstacle_file_exists() -> None:
    assert default_obstacle_file().is_file()


def test_load_world_columns() -> None:
    world = load_dubins_race_world()
    assert len(world.columns) >= 80
    assert world.vehicle_radius > 0.0


def test_build_occupancy_has_clearance() -> None:
    world = load_dubins_race_world()
    occ = build_race_occupancy(world)
    assert occ.clearance > world.vehicle_radius
