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
    # City-block multi-corridor layout: islands + props (not a packed maze).
    assert 20 <= len(world.structures) <= 40
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
    occ = RectStructureOccupancy(
        (rect,), vehicle_radius=0.42, clearance_margin=0.42
    )
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
        half_length=0.36,
        half_width=0.22,
        corner_sample_radius=0.05,
    )
    assert clearance > 0.0


def test_multi_corridor_layout_keeps_start_goal_clear() -> None:
    """Four aisle routes need clear pads at A/B (no dead-end packing)."""
    world = load_dubins_race_world()
    occ = build_race_occupancy(world)
    assert not occ.is_occupied(world.start_xy)
    assert not occ.is_occupied(world.goal_xy)
    assert len(world.structures) >= 20
