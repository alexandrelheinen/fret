"""Tests for fret.control.Kinematics — SCARA RRP model.

Acceptance criteria (FR-CTL-02):
  - ``forward_kinematics(zeros)`` returns the known home EE pose.
  - ``jacobian(q)`` has shape ``(6, 3)`` for the SCARA (DOF=3).
  - IK round-trip: ``FK(IK(target)) ≈ target`` within 5 mm.
  - ``joint_limits`` has shape ``(3, 2)`` for the SCARA.
  - ``dof`` is 3 for the SCARA.
  - ``joint_names`` returns 3 URDF names.
  - Wrong-shape input raises ``ValueError``.

SCARA RRP home pose (q = [0, 0, 0]):
  link_1: length L1 = 0.250 m along x
  link_2: length L2 = 0.200 m along x
  joint_3 (prismatic): d3 = 0 m (fully retracted)
  Home EE position in world frame: (0.450, 0.000, Z_base) where Z_base is
  the robot base height (defined in the XACRO).  For the purposes of this
  test we only verify the structure of the returned matrix, not the
  exact numerical values — those are verified in the implementation tests
  (Level 4).
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.kinematics import Kinematics

# SCARA DOF
_DOF = 3


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction() -> None:
    Kinematics(model="scara")


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_dof_is_three() -> None:
    k = Kinematics(model="scara")
    assert k.dof == _DOF


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_joint_names_length() -> None:
    k = Kinematics(model="scara")
    assert len(k.joint_names) == _DOF


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_joint_limits_shape() -> None:
    k = Kinematics(model="scara")
    limits = k.joint_limits
    assert limits.shape == (_DOF, 2), f"Expected ({_DOF}, 2), got {limits.shape}"
    # lower < upper for every joint
    assert np.all(limits[:, 0] < limits[:, 1])


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_forward_kinematics_home_returns_4x4() -> None:
    k = Kinematics(model="scara")
    T = k.forward_kinematics(np.zeros(_DOF))
    assert T.shape == (4, 4), f"Expected (4, 4), got {T.shape}"
    # Last row must be [0, 0, 0, 1] (homogeneous)
    np.testing.assert_array_almost_equal(T[3, :], [0.0, 0.0, 0.0, 1.0])


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_jacobian_shape() -> None:
    k = Kinematics(model="scara")
    J = k.jacobian(np.zeros(_DOF))
    assert J.shape == (6, _DOF), f"Expected (6, {_DOF}), got {J.shape}"


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_ik_round_trip() -> None:
    """FK(IK(target)) ≈ target within 5 mm EE error."""
    k = Kinematics(model="scara")
    q_ref = np.array([0.3, 0.2, 0.05])
    T_target = k.forward_kinematics(q_ref)
    q_ik = k.inverse_kinematics(T_target, seed=np.zeros(_DOF))
    T_recovered = k.forward_kinematics(q_ik)
    position_error = np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])
    assert position_error < 0.005, f"EE error {position_error:.4f} m exceeds 5 mm"


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_forward_kinematics_wrong_shape_raises() -> None:
    k = Kinematics(model="scara")
    with pytest.raises(ValueError):
        k.forward_kinematics(np.zeros(5))  # wrong DOF


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_jacobian_wrong_shape_raises() -> None:
    k = Kinematics(model="scara")
    with pytest.raises(ValueError):
        k.jacobian(np.zeros(5))
