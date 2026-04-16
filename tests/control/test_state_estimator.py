"""Tests for fret.control.StateEstimator.

Acceptance criteria (FR-CTL-01):
  - ``get_current_state()`` returns a ``RobotState`` after receiving a
    ``JointState`` message.
  - ``get_current_state()`` raises ``RuntimeError`` before the first message.
  - The returned ``RobotState.joint_names`` matches the robot model.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.state_estimator import StateEstimator
from fret.interfaces import RobotState


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction(mock_node: object, mock_kinematics: object) -> None:
    StateEstimator(node=mock_node, kinematics=mock_kinematics)


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_raises_before_first_message(
    mock_node: object, mock_kinematics: object
) -> None:
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    with pytest.raises(RuntimeError):
        estimator.get_current_state()


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_returns_robot_state(mock_node: object, mock_kinematics: object) -> None:
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    state = estimator.get_current_state()
    assert isinstance(state, RobotState)


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_joint_names_match_model(
    mock_node: object, mock_kinematics: object
) -> None:
    estimator = StateEstimator(node=mock_node, kinematics=mock_kinematics)
    state = estimator.get_current_state()
    assert len(state.joint_names) == 3  # SCARA DOF
