"""Tests for the PerceptionBridgeNode sampling logic.

Acceptance criteria (FR-PER-01, FR-PER-02, FR-PER-03):
  - ``_sample_box_surface`` returns an (N, 3) array with points on the
    surface within expected bounds.
  - ``_sample_cylinder_surface`` returns an (N, 3) array.
  - Box sampling produces at least ``density * total_surface_area * 0.5``
    points (generous lower bound accounting for rounding).
  - Cylinder sampling produces at least one point.
  - Pose translation is correctly applied (centroid near obstacle centre).
  - An empty obstacle list produces an empty cloud.

No ROS 2 installation is required to run these tests.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.ros.perception_bridge import (
    _apply_pose,
    _rotation_matrix_rpy,
    _sample_box_surface,
    _sample_cylinder_surface,
    _sample_obstacle_static,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_IDENTITY_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_DENSITY = 500.0  # lower density keeps tests fast


# ---------------------------------------------------------------------------
# _rotation_matrix_rpy
# ---------------------------------------------------------------------------


class TestRotationMatrix:
    """Unit tests for the RPY rotation matrix helper."""

    def test_identity(self) -> None:
        R = _rotation_matrix_rpy(0.0, 0.0, 0.0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_yaw_90(self) -> None:
        R = _rotation_matrix_rpy(0.0, 0.0, math.pi / 2)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        np.testing.assert_allclose(R, expected, atol=1e-12)

    def test_orthogonal(self) -> None:
        R = _rotation_matrix_rpy(0.3, 0.5, 1.2)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


# ---------------------------------------------------------------------------
# _apply_pose
# ---------------------------------------------------------------------------


class TestApplyPose:
    """Unit tests for the pose-transformation helper."""

    def test_translation_only(self) -> None:
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        pose = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        result = _apply_pose(pts, pose)
        expected = np.array([[1.0, 2.0, 3.0], [2.0, 2.0, 3.0]])
        np.testing.assert_allclose(result, expected, atol=1e-12)

    def test_rotation_then_translation(self) -> None:
        pts = np.array([[1.0, 0.0, 0.0]])
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2]
        result = _apply_pose(pts, pose)
        np.testing.assert_allclose(
            result, np.array([[0.0, 1.0, 0.0]]), atol=1e-12
        )


# ---------------------------------------------------------------------------
# _sample_box_surface
# ---------------------------------------------------------------------------


class TestSampleBoxSurface:
    """Unit tests for box surface sampling."""

    def test_returns_ndarray_shape(self) -> None:
        pts = _sample_box_surface([0.1, 0.1, 0.1], _IDENTITY_POSE, _DENSITY)
        assert isinstance(pts, np.ndarray)
        assert pts.ndim == 2
        assert pts.shape[1] == 3

    def test_minimum_point_count(self) -> None:
        sx, sy, sz = 0.1, 0.1, 0.1
        total_area = 2 * (sx * sy + sy * sz + sx * sz)
        pts = _sample_box_surface([sx, sy, sz], _IDENTITY_POSE, _DENSITY)
        min_expected = _DENSITY * total_area * 0.5
        assert pts.shape[0] >= min_expected

    def test_points_on_surface_identity_pose(self) -> None:
        """All points must lie on one of the six faces (within tolerance)."""
        sx, sy, sz = 0.10, 0.10, 0.10
        hx, hy, hz = sx / 2, sy / 2, sz / 2
        pts = _sample_box_surface([sx, sy, sz], _IDENTITY_POSE, _DENSITY)
        tol = 1e-9
        on_face = (
            (np.abs(pts[:, 0] - hx) < tol)
            | (np.abs(pts[:, 0] + hx) < tol)
            | (np.abs(pts[:, 1] - hy) < tol)
            | (np.abs(pts[:, 1] + hy) < tol)
            | (np.abs(pts[:, 2] - hz) < tol)
            | (np.abs(pts[:, 2] + hz) < tol)
        )
        assert on_face.all(), f"{(~on_face).sum()} points are NOT on any face"

    def test_centroid_near_pose_translation(self) -> None:
        pose = [0.25, 0.05, 0.05, 0.0, 0.0, 0.0]
        pts = _sample_box_surface([0.1, 0.1, 0.1], pose, _DENSITY)
        centroid = pts.mean(axis=0)
        np.testing.assert_allclose(centroid[:2], [0.25, 0.05], atol=0.02)

    def test_pose_translation_obstacle_a(self) -> None:
        pose = [0.25, 0.05, 0.05, 0.0, 0.0, 0.0]
        pts = _sample_box_surface([0.1, 0.1, 0.1], pose, _DENSITY)
        # All x must be in [0.20, 0.30], all y in [0.0, 0.10]
        assert pts[:, 0].min() >= 0.20 - 1e-9
        assert pts[:, 0].max() <= 0.30 + 1e-9
        assert pts[:, 1].min() >= 0.00 - 1e-9
        assert pts[:, 1].max() <= 0.10 + 1e-9

    def test_non_cubic_box(self) -> None:
        pts = _sample_box_surface([0.2, 0.1, 0.05], _IDENTITY_POSE, _DENSITY)
        assert pts.shape[0] > 0

    def test_dtype_float64(self) -> None:
        pts = _sample_box_surface([0.1, 0.1, 0.1], _IDENTITY_POSE, _DENSITY)
        assert pts.dtype == np.float64


# ---------------------------------------------------------------------------
# _sample_cylinder_surface
# ---------------------------------------------------------------------------


class TestSampleCylinderSurface:
    """Unit tests for cylinder surface sampling."""

    def test_returns_ndarray_shape(self) -> None:
        pts = _sample_cylinder_surface(0.05, 0.10, _IDENTITY_POSE, _DENSITY)
        assert isinstance(pts, np.ndarray)
        assert pts.ndim == 2
        assert pts.shape[1] == 3

    def test_at_least_one_point(self) -> None:
        pts = _sample_cylinder_surface(0.05, 0.10, _IDENTITY_POSE, _DENSITY)
        assert pts.shape[0] >= 1

    def test_centroid_near_pose_translation(self) -> None:
        pose = [0.5, 0.3, 0.1, 0.0, 0.0, 0.0]
        pts = _sample_cylinder_surface(0.05, 0.10, pose, _DENSITY)
        centroid = pts.mean(axis=0)
        np.testing.assert_allclose(centroid, [0.5, 0.3, 0.1], atol=0.02)

    def test_lateral_points_at_correct_radius(self) -> None:
        radius = 0.05
        pts = _sample_cylinder_surface(radius, 0.10, _IDENTITY_POSE, _DENSITY)
        # Points at |z| < half_len: must satisfy r ≈ radius or r ≤ radius (caps)
        half_len = 0.05
        lateral = pts[np.abs(pts[:, 2]) < half_len - 1e-9]
        if lateral.shape[0] > 0:
            r = np.sqrt(lateral[:, 0] ** 2 + lateral[:, 1] ** 2)
            assert r.max() <= radius + 1e-6

    def test_dtype_float64(self) -> None:
        pts = _sample_cylinder_surface(0.05, 0.10, _IDENTITY_POSE, _DENSITY)
        assert pts.dtype == np.float64


# ---------------------------------------------------------------------------
# _sample_obstacle_static
# ---------------------------------------------------------------------------


class TestSampleObstacleStatic:
    """Unit tests for the module-level obstacle sampling helper."""

    def test_box_obstacle(self) -> None:
        obs = {
            "type": "box",
            "size": [0.1, 0.1, 0.1],
            "pose": [0.25, 0.05, 0.05, 0.0, 0.0, 0.0],
        }
        pts = _sample_obstacle_static(obs, _DENSITY)
        assert pts.shape[1] == 3
        assert pts.shape[0] > 0

    def test_cylinder_obstacle(self) -> None:
        obs = {
            "type": "cylinder",
            "radius": 0.05,
            "length": 0.10,
            "pose": [0.1, 0.2, 0.0, 0.0, 0.0, 0.0],
        }
        pts = _sample_obstacle_static(obs, _DENSITY)
        assert pts.shape[1] == 3
        assert pts.shape[0] > 0

    def test_unknown_type_returns_empty(self) -> None:
        obs = {
            "type": "sphere",
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        pts = _sample_obstacle_static(obs, _DENSITY)
        assert pts.shape == (0, 3)


# ---------------------------------------------------------------------------
# PerceptionBridgeNode (pure-Python, no ROS)
# ---------------------------------------------------------------------------


class TestPerceptionBridgeNodePurePython:
    """Tests for PerceptionBridgeNode that do not require a ROS 2 runtime."""

    def _make_node_no_ros(
        self, obstacle_defs: list[dict], density: float = _DENSITY
    ) -> object:
        """Instantiate PerceptionBridgeNode in headless mode (no rclpy)."""
        import types

        from fret.ros import perception_bridge as pb

        # Temporarily monkey-patch _load_config and _resolve_config_path
        # to avoid filesystem / ament dependency, and stub out rclpy.
        fake_cfg = {
            "update_hz": 1.0,
            "surface_density": density,
            "obstacles": obstacle_defs,
        }

        node_obj = object.__new__(pb.PerceptionBridgeNode)
        node_obj._update_hz = float(fake_cfg["update_hz"])
        node_obj._surface_density = float(fake_cfg["surface_density"])
        node_obj._obstacle_defs = fake_cfg["obstacles"]
        node_obj._cloud_pts = node_obj._build_cloud()
        return node_obj

    def test_empty_config_empty_cloud(self) -> None:
        from fret.ros import perception_bridge as pb

        node = object.__new__(pb.PerceptionBridgeNode)
        node._surface_density = _DENSITY
        node._obstacle_defs = []
        cloud = node._build_cloud()
        assert cloud.shape == (0, 3)

    def test_single_box_produces_points(self) -> None:
        from fret.ros import perception_bridge as pb

        node = object.__new__(pb.PerceptionBridgeNode)
        node._surface_density = _DENSITY
        node._obstacle_defs = [
            {
                "type": "box",
                "size": [0.1, 0.1, 0.1],
                "pose": [0.25, 0.05, 0.05, 0.0, 0.0, 0.0],
            }
        ]
        cloud = node._build_cloud()
        assert cloud.ndim == 2
        assert cloud.shape[1] == 3
        assert cloud.shape[0] > 0

    def test_three_boxes_cloud_larger(self) -> None:
        from fret.ros import perception_bridge as pb

        node = object.__new__(pb.PerceptionBridgeNode)
        node._surface_density = _DENSITY
        node._obstacle_defs = [
            {
                "type": "box",
                "size": [0.1, 0.1, 0.1],
                "pose": [0.25, 0.05, 0.05, 0.0, 0.0, 0.0],
            },
            {
                "type": "box",
                "size": [0.1, 0.1, 0.1],
                "pose": [0.25, -0.05, 0.05, 0.0, 0.0, 0.0],
            },
            {
                "type": "box",
                "size": [0.1, 0.1, 0.1],
                "pose": [0.30, 0.00, 0.05, 0.0, 0.0, 0.0],
            },
        ]
        cloud = node._build_cloud()
        # Single-box count
        node_single = object.__new__(pb.PerceptionBridgeNode)
        node_single._surface_density = _DENSITY
        node_single._obstacle_defs = [node._obstacle_defs[0]]
        single = node_single._build_cloud()
        assert cloud.shape[0] > single.shape[0]
