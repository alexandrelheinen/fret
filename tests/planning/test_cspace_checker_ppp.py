"""Tests for fret.planning.PPPcSpaceChecker (T10-05, FR-GSP-02).

Acceptance criteria:
  - ``make_cspace_checker`` dispatches PPP vs SCARA.
  - EE-only and cargo-inclusive envelopes detect free / colliding configs.
  - Warehouse obstacle layout produces expected collisions.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control import GraspConfig, Kinematics
from fret.planning.cspace_checker import CSpaceChecker, make_cspace_checker
from fret.planning.cspace_checker_ppp import PPPCheckerConfig, PPPcSpaceChecker
from fret.planning.ppp_robot_envelope import ppp_body_envelopes
from fret.planning.ppp_obstacles import (
    BoxObstacle,
    BoxObstacleOccupancy,
    build_box_obstacle_occupancy,
    load_ppp_warehouse_obstacles,
    load_ppp_warehouse_preview_obstacles,
)


def _checker_with_boxes(
    boxes: list[BoxObstacle],
    *,
    include_cargo: bool = False,
) -> PPPcSpaceChecker:
    kin = Kinematics("ppp")
    occ = BoxObstacleOccupancy(boxes)
    return PPPcSpaceChecker(
        kin,
        occ,
        PPPCheckerConfig(include_cargo=include_cargo),
    )


def test_make_cspace_checker_dispatches_ppp() -> None:
    kin = Kinematics("ppp")
    occ = build_box_obstacle_occupancy([])
    checker = make_cspace_checker(kin, occ)
    assert isinstance(checker, PPPcSpaceChecker)


def test_make_cspace_checker_dispatches_scara(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    checker = make_cspace_checker(mock_kinematics, mock_occupancy)
    assert isinstance(checker, CSpaceChecker)


def test_free_above_single_box() -> None:
    box = BoxObstacle(10.0, 10.0, 0.0, 12.0, 12.0, 2.0)
    checker = _checker_with_boxes([box])
    q = np.array([1.0, 1.0, 4.0])
    assert checker.is_collision_free(q) is True
    assert checker.clearance(q) > 0.0


def test_collision_inside_single_box() -> None:
    box = BoxObstacle(10.0, 10.0, 0.0, 12.0, 12.0, 2.0)
    checker = _checker_with_boxes([box])
    q = np.array([11.0, 11.0, 1.0])
    assert checker.is_collision_free(q) is False
    assert checker.clearance(q) < 0.0


def test_cargo_extends_into_obstacle() -> None:
    """EE may be free while welded cargo penetrates an obstacle (FR-GSP-02)."""
    box = BoxObstacle(0.0, 0.0, 1.0, 60.0, 20.0, 1.8)
    grasp = GraspConfig(weld_offset=np.array([0.0, 0.0, -0.34]))
    kin = Kinematics("ppp")
    occ = BoxObstacleOccupancy([box])
    checker_ee = PPPcSpaceChecker(kin, occ, PPPCheckerConfig(include_cargo=False))
    checker_cargo = PPPcSpaceChecker(
        kin,
        occ,
        PPPCheckerConfig(include_cargo=True, grasp_config=grasp),
    )
    # EE z=2.0 keeps the Z column above the slab; cargo hangs into it.
    q = np.array([5.0, 5.0, 2.0])
    assert checker_ee.is_collision_free(q) is True
    assert checker_cargo.is_collision_free(q) is False


def test_warehouse_barrier_collision() -> None:
    boxes = load_ppp_warehouse_obstacles()
    checker = _checker_with_boxes(boxes)
    # Inside first width-crossing barrier (x=15..17, y=0..20, z=0..2.5)
    q_inside = np.array([16.0, 10.0, 1.0])
    assert checker.is_collision_free(q_inside) is False
    # Above the barrier
    q_above = np.array([16.0, 10.0, 4.0])
    assert checker.is_collision_free(q_above) is True


def test_shelf_rack_collision_with_z_column() -> None:
    """Z column must not pass through back-wall shelf racks."""
    boxes = load_ppp_warehouse_preview_obstacles()
    checker = _checker_with_boxes(boxes)
    # Inside rack_b footprint with column overlapping shelf height.
    q = np.array([9.0, 3.56, 1.0])
    assert checker.is_collision_free(q) is False
    # Same x but y shifted clear of rack depth.
    q_clear = np.array([9.0, 2.5, 1.0])
    assert checker.is_collision_free(q_clear) is True


def test_multi_body_envelope_samples_more_than_ee() -> None:
    envelopes = ppp_body_envelopes(np.array([1.0, 1.0, 1.0]))
    assert len(envelopes) >= 7


def test_wrong_dof_raises() -> None:
    checker = _checker_with_boxes([])
    with pytest.raises(ValueError):
        checker.is_collision_free(np.zeros(5))
