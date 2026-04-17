"""ROS 2 node entry point for the planner (stub — Level 3 only).

This module provides the ``planner_node`` executable entry point.  The full
ARCO-based implementation is deferred to Milestone 2.  For now the node
starts without errors and logs a warning that planning is not yet available.
"""

from __future__ import annotations


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``planner_node`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy
    import rclpy.node

    class _PlannerNodeStub(rclpy.node.Node):
        def __init__(self) -> None:
            super().__init__("planner_node")
            self.get_logger().warn(
                "PlannerNode: ARCO planning not implemented yet "
                "(Milestone 2).  Node is running as a no-op stub."
            )

    rclpy.init(args=args)
    node = _PlannerNodeStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
