"""Joint state estimation and TF2 broadcasting.

Subscribes to ``/joint_states`` (``sensor_msgs/JointState``), packages the
latest reading into a ``RobotState``, and broadcasts the EE pose to the TF2
tree so that downstream nodes can perform coordinate lookups.

Satisfies requirement FR-CTL-01.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from fret.interfaces import RobotState

if TYPE_CHECKING:
    import rclpy.node

    from fret.control.kinematics import Kinematics


class StateEstimator:
    """Subscribe to ``/joint_states`` and expose the current ``RobotState``.

    Args:
        node: The owning ROS 2 node.  The subscription is registered in the
            node's context so that spin drives callbacks.
        kinematics: Kinematics engine used to compute the EE pose for TF2
            broadcasting.
    """

    def __init__(self, node: rclpy.node.Node, kinematics: Kinematics) -> None:
        raise NotImplementedError

    def get_current_state(self) -> RobotState:
        """Return the latest joint-state snapshot.

        Returns:
            A ``RobotState`` populated from the most recently received
            ``JointState`` message.

        Raises:
            RuntimeError: If no ``JointState`` message has been received yet.
        """
        raise NotImplementedError
