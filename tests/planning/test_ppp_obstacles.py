"""Tests for fret.planning.ppp_obstacles (T10-08).

Acceptance criteria:
  - ``ppp_warehouse_obstacles.yml`` loads 10 ARCO-ported boxes.
  - ``boxes_to_point_cloud`` produces a non-empty point cloud.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.ppp_obstacles import (
    BoxObstacle,
    BoxObstacleOccupancy,
    boxes_to_point_cloud,
    default_obstacle_file,
    is_ppp_kinematics,
    load_ppp_warehouse_obstacles,
    load_ppp_warehouse_preview_obstacles,
    preview_obstacle_file,
)


def test_default_obstacle_file_exists() -> None:
    assert default_obstacle_file().is_file()


def test_load_ppp_warehouse_has_ten_boxes() -> None:
    boxes = load_ppp_warehouse_obstacles()
    assert len(boxes) == 10


def test_first_box_matches_arco_barrier() -> None:
    boxes = load_ppp_warehouse_obstacles()
    first = boxes[0]
    assert first.x_min == 15.0
    assert first.y_max == 20.0
    assert first.z_max == 2.5


def test_load_ppp_warehouse_preview_has_four_boxes() -> None:
    boxes = load_ppp_warehouse_preview_obstacles()
    assert len(boxes) == 4


def test_preview_obstacle_file_exists() -> None:
    assert preview_obstacle_file().is_file()


def test_boxes_to_point_cloud_non_empty() -> None:
    boxes = load_ppp_warehouse_obstacles()
    cloud = boxes_to_point_cloud(boxes, samples_per_edge=3)
    assert cloud.ndim == 2
    assert cloud.shape[1] == 3
    assert cloud.shape[0] > 0


def test_is_ppp_kinematics() -> None:
    assert is_ppp_kinematics(["joint_x", "joint_y", "joint_z"]) is True
    assert (
        is_ppp_kinematics(["joint_arm_0", "joint_arm_1", "joint_extension"])
        is False
    )


def test_box_obstacle_occupancy_inside_negative() -> None:
    box = BoxObstacle(10.0, 10.0, 0.0, 12.0, 12.0, 2.0)
    occ = BoxObstacleOccupancy([box])
    assert occ.clearance(np.array([11.0, 11.0, 1.0])) < 0.0


def test_box_obstacle_occupancy_outside_positive() -> None:
    box = BoxObstacle(10.0, 10.0, 0.0, 12.0, 12.0, 2.0)
    occ = BoxObstacleOccupancy([box])
    assert occ.clearance(np.array([1.0, 1.0, 4.0])) > 0.0


def test_default_contact_radius_is_one_point_five_centimetres() -> None:
    from fret.planning.ppp_obstacles import PPP_DEFAULT_CONTACT_RADIUS_M

    occ = BoxObstacleOccupancy([])
    assert occ._contact_radius == pytest.approx(PPP_DEFAULT_CONTACT_RADIUS_M)
    assert PPP_DEFAULT_CONTACT_RADIUS_M == pytest.approx(0.015)
