"""Unit tests for Dubins race obstacle loading (T11-02)."""

from __future__ import annotations

from fret.planning.dubins_obstacles import (
    RectObstacle,
    build_race_occupancy,
    circle_rect_clearance,
    default_obstacle_file,
    load_dubins_race_world,
)


def test_default_obstacle_file_exists() -> None:
    assert default_obstacle_file().is_file()


def test_load_world_structures() -> None:
    world = load_dubins_race_world()
    assert len(world.structures) >= 85
    assert world.vehicle_radius > 0.0


def test_build_occupancy_has_clearance() -> None:
    world = load_dubins_race_world()
    occ = build_race_occupancy(world)
    assert occ.clearance > world.vehicle_radius


def test_circle_rect_clearance_outside_positive() -> None:
    rect = RectObstacle(x=10.0, y=10.0, hx=0.5, hy=0.5, height=2.0)
    assert circle_rect_clearance(0.0, 0.0, 0.42, rect) > 0.0


def test_dead_end_parts_increase_structure_count() -> None:
    world = load_dubins_race_world()
    assert len(world.structures) >= 90
