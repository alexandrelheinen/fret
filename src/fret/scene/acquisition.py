"""Scene acquisition: PointCloud2 → world-frame obstacle payload.

Subscribes to ``/world_state`` (``sensor_msgs/PointCloud2``) published by
Gazebo, extracts XYZ obstacle geometry expressed in the ``world`` frame,
and exposes the result as an ``OccupancyUpdatePayload`` ready for the
planning layer.

The incoming cloud is assumed to already be in the ``world`` frame (as
published by the perception bridge).  TF2-based frame transformation will
be added in Level 4 for clouds arriving in other frames.

Satisfies requirements FR-SCN-01, FR-SCN-02, FR-SCN-04.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from fret.interfaces import OccupancyUpdatePayload

if TYPE_CHECKING:
    import rclpy.node


class SceneAcquisition:
    """Subscribe to Gazebo point cloud and deliver world-frame obstacle payloads.

    Args:
        node: The ROS 2 node that owns the subscription.  The subscription
            is created inside the node's context so that spin drives callbacks.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2  # type: ignore[import-untyped]

        self._node = node
        self._latest_payload: OccupancyUpdatePayload | None = None

        # RELIABLE + TRANSIENT_LOCAL so the last cloud survives late joiners.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        node.create_subscription(
            PointCloud2,
            "/world_state",
            self._point_cloud_callback,
            qos,
        )

    # ------------------------------------------------------------------
    # Internal callback
    # ------------------------------------------------------------------

    def _point_cloud_callback(self, msg: object) -> None:
        """Process an incoming ``PointCloud2`` message.

        Extracts XYZ points from the cloud (assumed to be in the ``world``
        frame) and stores an ``OccupancyUpdatePayload`` for later retrieval.

        Args:
            msg: A ``sensor_msgs/PointCloud2`` message.
        """
        try:
            from sensor_msgs_py.point_cloud2 import (  # type: ignore[import-untyped]
                read_points_numpy,
            )

            pts = read_points_numpy(
                msg,  # type: ignore[arg-type]
                field_names=["x", "y", "z"],
                skip_nans=True,
            )

            if pts.shape[0] == 0:
                xyz: np.ndarray = np.empty((0, 3), dtype=np.float64)
            else:
                xyz = np.column_stack(
                    [
                        pts["x"].astype(np.float64),
                        pts["y"].astype(np.float64),
                        pts["z"].astype(np.float64),
                    ]
                )

            stamp = msg.header.stamp  # type: ignore[attr-defined]
            timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

            self._latest_payload = OccupancyUpdatePayload(
                obstacle_points=xyz,
                timestamp=timestamp,
                frame_id="world",
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(  # type: ignore[attr-defined]
                f"SceneAcquisition: failed to process PointCloud2: {exc}"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_latest_payload(self) -> OccupancyUpdatePayload:
        """Return the most recently received obstacle payload.

        Returns:
            The latest ``OccupancyUpdatePayload`` with ``frame_id == "world"``.

        Raises:
            RuntimeError: If no message has been received yet.
        """
        if self._latest_payload is None:
            raise RuntimeError(
                "No PointCloud2 message received on /world_state yet. "
                "Ensure the perception bridge is running."
            )
        return self._latest_payload
