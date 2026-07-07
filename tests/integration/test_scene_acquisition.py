"""Integration tests: SceneAcquisition PointCloud2 → OccupancyUpdatePayload.

Verifies the acquisition pipeline end-to-end with a real ROS 2 context:
  - Publish a ``sensor_msgs/PointCloud2`` to ``/obstacle_cloud``.
  - Assert that ``SceneAcquisition.get_latest_payload()`` returns the
    expected ``OccupancyUpdatePayload``.

Requirements verified: FR-SCN-01, FR-SCN-02, FR-SCN-04.
"""

from __future__ import annotations

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2  # type: ignore[import-untyped]
from sensor_msgs_py.point_cloud2 import (
    create_cloud_xyz32,  # type: ignore[import-untyped]
)
from std_msgs.msg import Header  # type: ignore[import-untyped]

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.acquisition import SceneAcquisition

# Match SceneAcquisition subscription QoS (RELIABLE + TRANSIENT_LOCAL).
_ACQ_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_cloud(
    node: Node,
    points: list[tuple[float, float, float]],
    frame_id: str = "world",
) -> PointCloud2:
    header = Header()
    header.frame_id = frame_id
    header.stamp = node.get_clock().now().to_msg()
    return create_cloud_xyz32(header, points)


def _publish_and_spin(
    node: Node,
    publisher: object,
    msg: object,
    max_spins: int = 20,
    period: float = 0.05,
) -> None:
    for _ in range(max_spins):
        publisher.publish(msg)  # type: ignore[attr-defined]
        rclpy.spin_once(node, timeout_sec=period)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_scene_acquisition_construction(test_node: Node) -> None:
    """SceneAcquisition can be attached to a real ROS 2 node."""
    acq = SceneAcquisition(node=test_node)
    assert acq is not None


@pytest.mark.timeout(30)
def test_scene_acquisition_raises_before_message(test_node: Node) -> None:
    """Before any cloud arrives, get_latest_payload() raises RuntimeError."""
    acq = SceneAcquisition(node=test_node)
    with pytest.raises(RuntimeError):
        acq.get_latest_payload()


@pytest.mark.timeout(30)
def test_scene_acquisition_receives_cloud(test_node: Node) -> None:
    """After publishing /obstacle_cloud, get_latest_payload() returns a payload."""
    acq = SceneAcquisition(node=test_node)
    pub = test_node.create_publisher(PointCloud2, "/obstacle_cloud", _ACQ_QOS)
    cloud = _make_cloud(test_node, [(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)])
    _publish_and_spin(test_node, pub, cloud)
    payload = acq.get_latest_payload()
    assert isinstance(payload, OccupancyUpdatePayload)
    assert payload.frame_id == "world"
    assert payload.obstacle_points.shape == (2, 3)


@pytest.mark.timeout(30)
def test_scene_acquisition_point_values(test_node: Node) -> None:
    """The extracted point coordinates must match those in the published cloud."""
    pts = [(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)]
    acq = SceneAcquisition(node=test_node)
    pub = test_node.create_publisher(PointCloud2, "/obstacle_cloud", _ACQ_QOS)
    _publish_and_spin(test_node, pub, _make_cloud(test_node, pts))
    payload = acq.get_latest_payload()
    expected = np.array(pts, dtype=np.float64)
    np.testing.assert_allclose(payload.obstacle_points, expected, atol=1e-5)
