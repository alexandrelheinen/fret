"""Scene acquisition: PointCloud2 → world-frame obstacle payload.

Subscribes to ``/world_state`` (``sensor_msgs/PointCloud2``) published by
Gazebo, transforms every received cloud into the ``world`` coordinate frame,
and exposes the result as an ``OccupancyUpdatePayload`` ready for the
planning layer.

Satisfies requirements FR-SCN-01, FR-SCN-02, FR-SCN-04.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

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
        raise NotImplementedError

    def get_latest_payload(self) -> OccupancyUpdatePayload:
        """Return the most recently received obstacle payload.

        Returns:
            The latest ``OccupancyUpdatePayload`` with ``frame_id == "world"``.

        Raises:
            RuntimeError: If no message has been received yet.
        """
        raise NotImplementedError
