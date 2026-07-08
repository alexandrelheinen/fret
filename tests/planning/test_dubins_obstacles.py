"""Unit tests for Dubins race obstacle loading (T11-02)."""

from __future__ import annotations

import numpy as np

from fret.planning.dubins_obstacles import (
    RectObstacle,
    RectStructureOccupancy,
    build_race_occupancy,
    circle_rect_clearance,
    default_obstacle_file,
    load_dubins_race_world,
    vehicle_body_clearance,
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
    assert occ.clearance == world.vehicle_radius + world.clearance_margin
    assert not occ.is_occupied(np.array([0.0, 0.0]))


def test_circle_rect_clearance_outside_positive() -> None:
    rect = RectObstacle(x=10.0, y=10.0, hx=0.5, hy=0.5, height=2.0)
    assert circle_rect_clearance(0.0, 0.0, 0.42, rect) > 0.0


def test_rect_structure_occupancy_blocks_near_wall() -> None:
    rect = RectObstacle(x=10.0, y=10.0, hx=0.5, hy=0.5, height=2.0)
    occ = RectStructureOccupancy((rect,), vehicle_radius=0.42, clearance_margin=0.42)
    assert occ.is_occupied(np.array([10.0, 10.0]))
    assert not occ.is_occupied(np.array([0.0, 0.0]))


def test_vehicle_body_clearance_samples_corners() -> None:
    rect = RectObstacle(x=10.0, y=10.0, hx=0.5, hy=0.5, height=2.0)
    clearance = vehicle_body_clearance(
        0.0,
        0.0,
        0.0,
        (rect,),
        vehicle_radius=0.42,
    )
    assert clearance > 0.0


def test_dead_end_parts_increase_structure_count() -> None:
    world = load_dubins_race_world()
    assert len(world.structures) >= 90
