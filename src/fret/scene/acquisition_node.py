"""ROS 2 node entry point for scene acquisition.

Wraps ``SceneAcquisition`` in a live ROS 2 node that subscribes to
``/world_state`` (PointCloud2) and stores the latest obstacle payload.

This module provides the ``scene_acquisition_node`` executable entry point.
"""

from __future__ import annotations


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``scene_acquisition_node`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy
    import rclpy.node

    from fret.scene.acquisition import SceneAcquisition

    class _SceneAcquisitionNode(rclpy.node.Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("scene_acquisition_node")
            self._acquisition = SceneAcquisition(node=self)
            self.get_logger().info("SceneAcquisitionNode ready.")

    rclpy.init(args=args)
    node = _SceneAcquisitionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
