"""Tests for MuJoCo-backed PPP collision checking."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control import Kinematics
from fret.planning.cspace_checker import make_cspace_checker
from fret.planning.cspace_checker_mujoco import MujocoPPPCollisionChecker
from fret.planning.ppp_obstacles import build_box_obstacle_occupancy


@pytest.fixture(scope="module")
def mujoco_available() -> None:
    pytest.importorskip("mujoco")


def test_make_cspace_checker_dispatches_mujoco(mujoco_available: None) -> None:
    kin = Kinematics("ppp")
    occ = build_box_obstacle_occupancy([])
    checker = make_cspace_checker(
        kin, occ, collision_backend="mujoco", scenario="ppp_warehouse"
    )
    assert isinstance(checker, MujocoPPPCollisionChecker)


def test_mujoco_free_configuration(mujoco_available: None) -> None:
    kin = Kinematics("ppp")
    checker = make_cspace_checker(
        kin,
        build_box_obstacle_occupancy([]),
        collision_backend="mujoco",
    )
    q = np.array([2.0, 1.0, 2.4])
    assert checker.is_collision_free(q) is True
    assert checker.clearance(q) > 0.0


def test_mujoco_detects_obstacle_penetration(mujoco_available: None) -> None:
    kin = Kinematics("ppp")
    checker = make_cspace_checker(
        kin,
        build_box_obstacle_occupancy([]),
        collision_backend="mujoco",
    )
    # Inside obs_b collision box (pos 7.0 4.0 0.5, size 0.7 0.7 0.5).
    q = np.array([7.0, 4.0, 0.5])
    assert checker.is_collision_free(q) is False
    assert checker.clearance(q) < 0.0


def test_mujoco_include_cargo_flag(mujoco_available: None) -> None:
    """Cargo geom is optional in MuJoCo contact checks (FR-GSP-02)."""
    kin = Kinematics("ppp")
    checker_ee = make_cspace_checker(
        kin,
        build_box_obstacle_occupancy([]),
        collision_backend="mujoco",
        include_cargo=False,
    )
    checker_cargo = make_cspace_checker(
        kin,
        build_box_obstacle_occupancy([]),
        collision_backend="mujoco",
        include_cargo=True,
    )
    assert isinstance(checker_ee, MujocoPPPCollisionChecker)
    assert checker_ee.include_cargo is False
    assert checker_cargo.include_cargo is True
    assert len(checker_cargo._cargo_geom_ids) > 0
