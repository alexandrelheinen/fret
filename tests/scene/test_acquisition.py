"""Tests for the scene acquisition layer (fret.scene.SceneAcquisition).

Acceptance criteria (FR-SCN-01, FR-SCN-02, FR-SCN-04):
  - Construction registers a /world_state subscription on the given node.
  - ``get_latest_payload()`` raises ``RuntimeError`` before any message arrives.
  - After the callback is invoked, ``get_latest_payload()`` returns an
    ``OccupancyUpdatePayload`` with ``frame_id == "world"``.
  - ``payload.obstacle_points`` has shape ``(N, 3)`` with N ≥ 0.
  - The timestamp in the payload matches the message stamp.

The callback is exercised by building a real ``sensor_msgs/PointCloud2``
message via ``sensor_msgs_py.point_cloud2.create_cloud_xyz32`` so that the
extraction logic (``read_points_numpy``) is genuinely tested.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.acquisition import SceneAcquisition

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_point_cloud(
    points: list[tuple[float, float, float]],
    frame_id: str = "world",
    sec: int = 3,
    nanosec: int = 0,
) -> object:
    """Return a real ``sensor_msgs/PointCloud2`` for the given XYZ points."""
    from builtin_interfaces.msg import Time  # type: ignore[import-untyped]
    from sensor_msgs_py.point_cloud2 import (  # type: ignore[import-untyped]
        create_cloud_xyz32,
    )
    from std_msgs.msg import Header  # type: ignore[import-untyped]

    header = Header()
    header.frame_id = frame_id
    header.stamp = Time(sec=sec, nanosec=nanosec)
    return create_cloud_xyz32(header, points)


def _get_subscription_callback(mock_node: MagicMock) -> object:
    """Return the callback registered with ``node.create_subscription``."""
    assert mock_node.create_subscription.called
    return mock_node.create_subscription.call_args[0][2]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_construction_with_node(mock_node: MagicMock) -> None:
    """SceneAcquisition must accept a ROS 2 node handle."""
    acq = SceneAcquisition(node=mock_node)
    assert acq is not None


def test_subscribes_to_world_state(mock_node: MagicMock) -> None:
    """Constructor must register a subscription on /world_state."""
    SceneAcquisition(node=mock_node)
    assert mock_node.create_subscription.called
    topic = mock_node.create_subscription.call_args[0][1]
    assert topic == "/world_state"


def test_raises_before_first_message(mock_node: MagicMock) -> None:
    """get_latest_payload() must raise RuntimeError before first message."""
    acq = SceneAcquisition(node=mock_node)
    with pytest.raises(RuntimeError):
        acq.get_latest_payload()


def test_get_latest_payload_returns_payload(mock_node: MagicMock) -> None:
    """After callback, get_latest_payload() returns OccupancyUpdatePayload."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    cloud = _make_point_cloud([(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)])
    callback(cloud)
    payload = acq.get_latest_payload()
    assert isinstance(payload, OccupancyUpdatePayload)


def test_frame_id_is_world(mock_node: MagicMock) -> None:
    """All payloads must carry frame_id == 'world' (FR-SCN-02)."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud([(0.1, 0.2, 0.3)]))
    payload = acq.get_latest_payload()
    assert payload.frame_id == "world"


def test_obstacle_points_shape(mock_node: MagicMock) -> None:
    """obstacle_points must have shape (N, 3) with N == number of points."""
    n_points = 4
    pts = [(float(i), float(i), float(i)) for i in range(n_points)]
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud(pts))
    payload = acq.get_latest_payload()
    assert payload.obstacle_points.ndim == 2
    assert payload.obstacle_points.shape == (n_points, 3)


def test_obstacle_points_are_float64(mock_node: MagicMock) -> None:
    """obstacle_points dtype must be float64."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud([(0.1, 0.2, 0.3)]))
    payload = acq.get_latest_payload()
    assert payload.obstacle_points.dtype == np.float64


def test_obstacle_points_values(mock_node: MagicMock) -> None:
    """obstacle_points values must match the input cloud."""
    pts = [(0.1, 0.2, 0.3), (0.4, 0.5, 0.6)]
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud(pts))
    payload = acq.get_latest_payload()
    expected = np.array(pts, dtype=np.float64)
    np.testing.assert_allclose(payload.obstacle_points, expected, atol=1e-5)


def test_timestamp_matches_message(mock_node: MagicMock) -> None:
    """Payload timestamp must equal sec + nanosec * 1e-9."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud([(0.0, 0.0, 0.0)], sec=5, nanosec=250_000_000))
    payload = acq.get_latest_payload()
    assert payload.timestamp == pytest.approx(5.25)


def test_empty_cloud_gives_zero_row_array(mock_node: MagicMock) -> None:
    """An empty PointCloud2 must produce obstacle_points with shape (0, 3)."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud([]))
    payload = acq.get_latest_payload()
    assert payload.obstacle_points.shape == (0, 3)


def test_latest_payload_replaced_on_new_message(mock_node: MagicMock) -> None:
    """Each callback call must overwrite the stored payload."""
    acq = SceneAcquisition(node=mock_node)
    callback = _get_subscription_callback(mock_node)
    callback(_make_point_cloud([(0.1, 0.2, 0.3)]))
    callback(_make_point_cloud([(9.9, 8.8, 7.7)]))
    payload = acq.get_latest_payload()
    np.testing.assert_allclose(
        payload.obstacle_points[0], [9.9, 8.8, 7.7], atol=1e-4
    )
