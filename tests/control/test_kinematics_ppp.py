"""Tests for fret.control.Kinematics — PPP gantry model (v1.0).

Acceptance criteria (T10-01, FR-SYS-01, FR-CTL-02, FR-PLN-02):
  - ``Kinematics(model="ppp")`` constructs with DOF 3.
  - ``joint_names`` returns PPP prismatic joint names.
  - ``joint_limits`` match the v1.0 operational envelope.
  - FK/IK implement the identity map ``q ≡ p_ee``.
  - ``jacobian(q)`` has shape ``(6, 3)`` with prismatic columns.
  - Wrong-shape input raises ``ValueError``.
  - Out-of-envelope IK targets raise ``RuntimeError``.

PPP model parameters (from docs/robots/ppp.md):
  Workspace: X [0, 60], Y [0, 20], Z [0, 6] m
  Joint names: joint_x, joint_y, joint_z
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.kinematics import Kinematics

_DOF = 3
_POS_TOL_M = 0.001  # 1 mm — identity map should be exact within float noise

_X_LIMITS = (0.0, 60.0)
_Y_LIMITS = (0.0, 20.0)
_Z_LIMITS = (0.0, 6.0)


def test_construction() -> None:
    Kinematics(model="ppp")


def test_dof_is_three() -> None:
    k = Kinematics(model="ppp")
    assert k.dof == _DOF


def test_joint_names() -> None:
    k = Kinematics(model="ppp")
    assert k.joint_names == ["joint_x", "joint_y", "joint_z"]


def test_joint_limits_shape() -> None:
    k = Kinematics(model="ppp")
    limits = k.joint_limits
    assert limits.shape == (_DOF, 2)
    assert np.all(limits[:, 0] < limits[:, 1])


def test_joint_limits_values() -> None:
    k = Kinematics(model="ppp")
    lim = k.joint_limits
    assert lim[0, 0] == pytest.approx(_X_LIMITS[0])
    assert lim[0, 1] == pytest.approx(_X_LIMITS[1])
    assert lim[1, 0] == pytest.approx(_Y_LIMITS[0])
    assert lim[1, 1] == pytest.approx(_Y_LIMITS[1])
    assert lim[2, 0] == pytest.approx(_Z_LIMITS[0])
    assert lim[2, 1] == pytest.approx(_Z_LIMITS[1])


def test_forward_kinematics_returns_4x4() -> None:
    k = Kinematics(model="ppp")
    T = k.forward_kinematics(np.array([1.0, 2.0, 3.0]))
    assert T.shape == (4, 4)
    np.testing.assert_array_almost_equal(T[3, :], [0.0, 0.0, 0.0, 1.0])


def test_forward_kinematics_identity_position() -> None:
    """FK maps joint positions directly to EE position (identity map)."""
    k = Kinematics(model="ppp")
    q = np.array([10.0, 5.0, 2.5])
    T = k.forward_kinematics(q)
    assert T[0, 3] == pytest.approx(10.0)
    assert T[1, 3] == pytest.approx(5.0)
    assert T[2, 3] == pytest.approx(2.5)


def test_forward_kinematics_identity_rotation() -> None:
    k = Kinematics(model="ppp")
    T = k.forward_kinematics(np.array([1.0, 1.0, 1.0]))
    np.testing.assert_array_almost_equal(T[:3, :3], np.eye(3))


def test_jacobian_shape() -> None:
    k = Kinematics(model="ppp")
    J = k.jacobian(np.array([1.0, 2.0, 3.0]))
    assert J.shape == (6, _DOF)


def test_jacobian_prismatic_columns() -> None:
    """Each prismatic joint maps unit joint velocity to a world-axis linear twist."""
    k = Kinematics(model="ppp")
    J = k.jacobian(np.array([5.0, 5.0, 3.0]))
    expected = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_array_almost_equal(J, expected)


def test_ik_round_trip() -> None:
    k = Kinematics(model="ppp")
    q_ref = np.array([15.0, 8.0, 4.0])
    T_target = k.forward_kinematics(q_ref)
    q_ik = k.inverse_kinematics(T_target)
    T_recovered = k.forward_kinematics(q_ik)
    position_error = np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])
    assert position_error < _POS_TOL_M


def test_ik_multiple_configurations() -> None:
    k = Kinematics(model="ppp")
    configs = [
        [0.0, 0.0, 0.0],
        [30.0, 10.0, 3.0],
        [60.0, 20.0, 6.0],
        [5.5, 12.3, 1.7],
    ]
    for q_ref_list in configs:
        q_ref = np.array(q_ref_list)
        T_target = k.forward_kinematics(q_ref)
        q_ik = k.inverse_kinematics(T_target)
        T_recovered = k.forward_kinematics(q_ik)
        err = np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])
        assert err < _POS_TOL_M, f"q={q_ref} → error {err:.4f} m"


def test_ik_out_of_envelope_raises() -> None:
    k = Kinematics(model="ppp")
    pose = np.eye(4)
    pose[0, 3] = 70.0  # beyond X limit
    with pytest.raises(RuntimeError):
        k.inverse_kinematics(pose)


def test_forward_kinematics_wrong_shape_raises() -> None:
    k = Kinematics(model="ppp")
    with pytest.raises(ValueError):
        k.forward_kinematics(np.zeros(5))


def test_jacobian_wrong_shape_raises() -> None:
    k = Kinematics(model="ppp")
    with pytest.raises(ValueError):
        k.jacobian(np.zeros(5))
