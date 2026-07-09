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
from fret.config_loader import load_algorithm_config
from fret.planning.ppp_obstacles import (
    BoxObstacle,
    BoxObstacleOccupancy,
    build_box_obstacle_occupancy,
    load_ppp_warehouse_obstacles,
    load_ppp_warehouse_preview_obstacles,
)

_CONTACT_RADIUS = float(load_algorithm_config("planning/ppp.yml")["contact_radius"])


def _checker_with_boxes(
    boxes: list[BoxObstacle],
    *,
    include_cargo: bool = False,
) -> PPPcSpaceChecker:
    kin = Kinematics("ppp")
    occ = BoxObstacleOccupancy(boxes, contact_radius=_CONTACT_RADIUS)
    return PPPcSpaceChecker(
        kin,
        occ,
        PPPCheckerConfig(include_cargo=include_cargo),
    )


def test_make_cspace_checker_dispatches_ppp() -> None:
    kin = Kinematics("ppp")
    occ = build_box_obstacle_occupancy([], contact_radius=_CONTACT_RADIUS)
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
    from fret.config_loader import load_algorithm_config
    from fret.control.grasp_magnet import parse_grasp_config

    box = BoxObstacle(0.0, 0.0, 1.0, 60.0, 20.0, 1.8)
    grasp = parse_grasp_config(load_algorithm_config("grasp/ppp_warehouse.yml"))
    kin = Kinematics("ppp")
    occ = BoxObstacleOccupancy([box], contact_radius=_CONTACT_RADIUS)
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


def test_shifted_obstacle_blocks_far_aisle() -> None:
    """Shifted obs_b must still constrain the far-side transit lane."""
    boxes = load_ppp_warehouse_preview_obstacles()
    checker = _checker_with_boxes(boxes)
    q = np.array([7.0, 4.0, 0.5])
    assert checker.is_collision_free(q) is False
    q_clear = np.array([5.0, 2.0, 2.0])
    assert checker.is_collision_free(q_clear) is True


def test_multi_body_envelope_samples_more_than_ee() -> None:
    envelopes = ppp_body_envelopes(np.array([1.0, 1.0, 1.0]))
    assert len(envelopes) >= 7


def test_wrong_dof_raises() -> None:
    checker = _checker_with_boxes([])
    with pytest.raises(ValueError):
        checker.is_collision_free(np.zeros(5))
