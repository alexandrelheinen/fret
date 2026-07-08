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


def test_mujoco_detects_shelf_penetration(mujoco_available: None) -> None:
    kin = Kinematics("ppp")
    checker = make_cspace_checker(
        kin,
        build_box_obstacle_occupancy([]),
        collision_backend="mujoco",
    )
    q = np.array([9.0, 3.56, 1.0])
    assert checker.is_collision_free(q) is False
    assert checker.clearance(q) < 0.0
