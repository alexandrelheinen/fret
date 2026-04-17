"""Integration tests: Kinematics ↔ StateEstimator round-trip.

Verifies that real ``rclpy`` infrastructure can be used to publish a
``JointState`` message and have ``StateEstimator`` process it correctly.

Test strategy (per project decision — integration-tests.yml):
  - Spin real ROS 2 nodes using rclpy.
  - Inject simulated ``sensor_msgs/JointState`` messages.
  - Assert that ``StateEstimator.get_current_state()`` returns a
    ``RobotState`` with values that match the published message.

Requirements verified: FR-CTL-01, FR-CTL-02.
"""

from __future__ import annotations

import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]

from fret.control.kinematics import Kinematics
from fret.control.state_estimator import StateEstimator
from fret.interfaces import RobotState

_JOINT_NAMES = ["joint_arm_0", "joint_arm_1", "joint_extension"]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _publish_and_spin(
    node: Node,
    publisher: object,
    msg: object,
    max_spins: int = 20,
    period: float = 0.05,
) -> None:
    """Publish a message and spin until delivered or timeout."""
    for _ in range(max_spins):
        publisher.publish(msg)  # type: ignore[attr-defined]
        rclpy.spin_once(node, timeout_sec=period)


def _make_joint_state(
    node: Node,
    positions: list[float],
    velocities: list[float] | None = None,
) -> JointState:
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = _JOINT_NAMES
    msg.position = positions
    msg.velocity = velocities or [0.0] * len(positions)
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_kinematics_construction_in_ros_context(ros_context: object) -> None:
    """Kinematics must be constructable inside a live ROS context."""
    k = Kinematics("scara")
    assert k.dof == 3
    assert k.joint_names == _JOINT_NAMES


@pytest.mark.timeout(30)
def test_state_estimator_construction(test_node: Node) -> None:
    """StateEstimator can be attached to a real ROS 2 node."""
    k = Kinematics("scara")
    estimator = StateEstimator(node=test_node, kinematics=k)
    assert estimator is not None


@pytest.mark.timeout(30)
def test_state_estimator_raises_before_message(test_node: Node) -> None:
    """Before any message arrives, get_current_state() raises RuntimeError."""
    k = Kinematics("scara")
    estimator = StateEstimator(node=test_node, kinematics=k)
    with pytest.raises(RuntimeError):
        estimator.get_current_state()


@pytest.mark.timeout(30)
def test_state_estimator_receives_joint_state(test_node: Node) -> None:
    """After publishing to /joint_states, the estimator returns a RobotState."""
    k = Kinematics("scara")
    estimator = StateEstimator(node=test_node, kinematics=k)

    pub = test_node.create_publisher(JointState, "/joint_states", 10)
    msg = _make_joint_state(test_node, [0.1, 0.2, 0.05])

    _publish_and_spin(test_node, pub, msg)

    state = estimator.get_current_state()
    assert isinstance(state, RobotState)
    np.testing.assert_allclose(
        state.joint_positions, [0.1, 0.2, 0.05], atol=1e-6
    )
    assert state.joint_names == _JOINT_NAMES


@pytest.mark.timeout(30)
def test_state_estimator_updates_on_new_message(test_node: Node) -> None:
    """The estimator must reflect the most recently published state."""
    k = Kinematics("scara")
    estimator = StateEstimator(node=test_node, kinematics=k)

    pub = test_node.create_publisher(JointState, "/joint_states", 10)

    _publish_and_spin(
        test_node, pub, _make_joint_state(test_node, [0.0, 0.0, 0.0])
    )
    _publish_and_spin(
        test_node, pub, _make_joint_state(test_node, [0.5, 0.5, 0.1])
    )

    state = estimator.get_current_state()
    np.testing.assert_allclose(
        state.joint_positions, [0.5, 0.5, 0.1], atol=1e-6
    )


@pytest.mark.timeout(30)
def test_kinematics_fk_ik_round_trip_in_context(ros_context: object) -> None:
    """FK ↔ IK round-trip must hold inside a live ROS context."""
    k = Kinematics("scara")
    q_ref = np.array([0.3, 0.2, 0.05])
    T = k.forward_kinematics(q_ref)
    q_ik = k.inverse_kinematics(T)
    T_recovered = k.forward_kinematics(q_ik)
    err = np.linalg.norm(T_recovered[:3, 3] - T[:3, 3])
    assert err < 0.005, f"EE error {err:.4f} m exceeds 5 mm"
