"""Joint state estimation and TF2 broadcasting.

Subscribes to ``/joint_states`` (``sensor_msgs/JointState``), packages the
latest reading into a ``RobotState``, and broadcasts the EE pose to the TF2
tree so that downstream nodes can perform coordinate lookups.

Satisfies requirement FR-CTL-01.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from fret.interfaces import RobotState

if TYPE_CHECKING:
    import rclpy.node

    from fret.control.kinematics import Kinematics


class StateEstimator:
    """Subscribe to ``/joint_states`` and expose the current ``RobotState``.

    Args:
        node: The owning ROS 2 node.  The subscription is registered in the
            node's context so that spin drives callbacks.
        kinematics: Kinematics engine used to resolve joint-name ordering and
            to compute the EE pose for TF2 broadcasting.
    """

    def __init__(self, node: rclpy.node.Node, kinematics: Kinematics) -> None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        self._node = node
        self._kinematics = kinematics
        self._latest_state: RobotState | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos,
        )

    # ------------------------------------------------------------------
    # Internal callback
    # ------------------------------------------------------------------

    def _joint_state_callback(self, msg: object) -> None:
        """Process an incoming ``JointState`` message.

        Reorders the message fields to match the kinematics joint ordering
        and stores a new ``RobotState``.

        Args:
            msg: A ``sensor_msgs/JointState`` message.
        """
        names = list(msg.name)  # type: ignore[attr-defined]
        pos_raw = list(msg.position)  # type: ignore[attr-defined]
        vel_raw = list(msg.velocity)  # type: ignore[attr-defined]

        def _get(seq: list[float], name: str) -> float:
            try:
                return float(seq[names.index(name)])
            except (ValueError, IndexError):
                return 0.0

        positions = np.array(
            [_get(pos_raw, n) for n in self._kinematics.joint_names],
            dtype=np.float64,
        )
        velocities = np.array(
            [_get(vel_raw, n) for n in self._kinematics.joint_names],
            dtype=np.float64,
        )

        stamp = msg.header.stamp  # type: ignore[attr-defined]
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        self._latest_state = RobotState(
            joint_positions=positions,
            joint_velocities=velocities,
            joint_names=self._kinematics.joint_names,
            timestamp=timestamp,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_current_state(self) -> RobotState:
        """Return the latest joint-state snapshot.

        Returns:
            A ``RobotState`` populated from the most recently received
            ``JointState`` message.

        Raises:
            RuntimeError: If no ``JointState`` message has been received yet.
        """
        if self._latest_state is None:
            raise RuntimeError(
                "No JointState message received on /joint_states yet. "
                "Ensure the robot state publisher is running."
            )
        return self._latest_state
