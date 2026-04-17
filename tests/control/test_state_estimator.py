"""Tests for fret.control.StateEstimator.

Acceptance criteria (FR-CTL-01):
  - ``get_current_state()`` raises ``RuntimeError`` before the first message.
  - After callback invocation, ``get_current_state()`` returns a ``RobotState``.
  - ``RobotState.joint_names`` matches the kinematics joint ordering.
  - ``RobotState.joint_positions`` is populated from the message.
  - Joints absent from the message are reported as velocity = 0.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from fret.control.state_estimator import StateEstimator
from fret.interfaces import RobotState

_JOINT_NAMES = ["joint_arm_0", "joint_arm_1", "joint_extension"]
_DOF = 3


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_joint_state_msg(
    names: list[str] | None = None,
    positions: list[float] | None = None,
    velocities: list[float] | None = None,
    sec: int = 1,
    nanosec: int = 500_000_000,
) -> MagicMock:
    """Build a mock ``sensor_msgs/JointState`` message."""
    msg = MagicMock()
    msg.name = names if names is not None else _JOINT_NAMES
    msg.position = positions if positions is not None else [0.1, 0.2, 0.05]
    msg.velocity = velocities if velocities is not None else [0.0, 0.0, 0.0]
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    return msg


def _get_subscription_callback(mock_node: MagicMock) -> object:
    """Return the callback registered with ``node.create_subscription``."""
    assert mock_node.create_subscription.called, "No subscription was created"
    # Positional args: (msg_type, topic, callback, qos)
    return mock_node.create_subscription.call_args[0][2]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_construction(mock_node: object, mock_kinematics: object) -> None:
    """StateEstimator can be constructed with a node and kinematics."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    assert estimator is not None


def test_subscribes_to_joint_states(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """Constructor must register a subscription on /joint_states."""
    StateEstimator(node=mock_node, kinematics=mock_kinematics)
    assert mock_node.create_subscription.called
    topic = mock_node.create_subscription.call_args[0][1]
    assert topic == "/joint_states"


def test_raises_before_first_message(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """get_current_state() must raise RuntimeError before any message arrives."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    with pytest.raises(RuntimeError):
        estimator.get_current_state()


def test_returns_robot_state_after_callback(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """After the callback is invoked, get_current_state() returns a RobotState."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    callback(_make_joint_state_msg())
    state = estimator.get_current_state()
    assert isinstance(state, RobotState)


def test_joint_positions_match_message(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """joint_positions must reflect the message values."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    msg = _make_joint_state_msg(positions=[0.1, 0.2, 0.05])
    callback(msg)
    state = estimator.get_current_state()
    assert state.joint_positions.shape == (_DOF,)
    np.testing.assert_allclose(state.joint_positions, [0.1, 0.2, 0.05])


def test_joint_names_match_kinematics(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """joint_names must be ordered by kinematics, not the message."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    # Scramble message order
    msg = _make_joint_state_msg(
        names=["joint_extension", "joint_arm_1", "joint_arm_0"],
        positions=[0.05, 0.2, 0.1],
    )
    callback(msg)
    state = estimator.get_current_state()
    assert state.joint_names == _JOINT_NAMES
    # Positions must be in kinematics order (arm_0=0.1, arm_1=0.2, ext=0.05)
    np.testing.assert_allclose(state.joint_positions, [0.1, 0.2, 0.05])


def test_timestamp_is_set(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """Timestamp must equal sec + nanosec * 1e-9."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    callback(_make_joint_state_msg(sec=2, nanosec=0))
    state = estimator.get_current_state()
    assert state.timestamp == pytest.approx(2.0)


def test_velocity_zeros_when_absent(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """Missing velocity entries default to 0.0."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    msg = _make_joint_state_msg(velocities=[])
    callback(msg)
    state = estimator.get_current_state()
    np.testing.assert_array_equal(state.joint_velocities, np.zeros(_DOF))


def test_latest_state_is_replaced_on_new_message(
    mock_node: MagicMock, mock_kinematics: object
) -> None:
    """Each callback invocation should overwrite the stored state."""
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    callback = _get_subscription_callback(mock_node)
    callback(_make_joint_state_msg(positions=[0.1, 0.2, 0.05]))
    callback(_make_joint_state_msg(positions=[0.5, 0.6, 0.1]))
    state = estimator.get_current_state()
    np.testing.assert_allclose(state.joint_positions, [0.5, 0.6, 0.1])
