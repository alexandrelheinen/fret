"""Unit tests for Dubins SE(2) kinematics (T11-01)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.control.kinematics import Kinematics
from fret.control.kinematics_dubins import DubinsKinematics


def test_dubins_dof_and_joint_names() -> None:
    kin = DubinsKinematics()
    assert kin.dof == 3
    assert kin.joint_names == ["joint_x", "joint_y", "joint_yaw"]


def test_dubins_facade_registers_model() -> None:
    kin = Kinematics("dubins")
    assert kin.dof == 3


def test_forward_kinematics_pose_matrix() -> None:
    kin = DubinsKinematics()
    q = np.array([3.0, 4.0, math.pi / 2], dtype=np.float64)
    pose = kin.forward_kinematics(q)
    assert pose.shape == (4, 4)
    assert pose[0, 3] == pytest.approx(3.0)
    assert pose[1, 3] == pytest.approx(4.0)
    assert pose[0, 0] == pytest.approx(0.0, abs=1e-9)
    assert pose[0, 1] == pytest.approx(-1.0, abs=1e-9)


def test_inverse_kinematics_round_trip() -> None:
    kin = DubinsKinematics()
    q0 = np.array([5.0, 6.0, 0.25], dtype=np.float64)
    q1 = kin.inverse_kinematics(kin.forward_kinematics(q0))
    np.testing.assert_allclose(q1, q0, atol=1e-9)
