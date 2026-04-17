"""Tests for fret.control.Kinematics — SCARA RRP model.

Acceptance criteria (FR-CTL-02):
  - ``forward_kinematics(zeros)`` returns the known home EE pose.
  - ``jacobian(q)`` has shape ``(6, 3)`` for the SCARA (DOF=3).
  - IK round-trip: ``FK(IK(target)) ≈ target`` within 5 mm.
  - ``joint_limits`` has shape ``(3, 2)`` for the SCARA.
  - ``dof`` is 3 for the SCARA.
  - ``joint_names`` returns 3 URDF names.
  - Wrong-shape input raises ``ValueError``.

SCARA RRP model parameters (from src/fret/urdf/scara.xacro):
  L1 = 0.325 m, L2 = 0.275 m
  j1_z (link_0_cylinder_height) = 0.1665 m
  j2_z (link_1_height) = 0.0715 m
  z_base = 0.238 m
  joint limits: j1 ±132°, j2 ±150°, j3 [0, 0.2 m]
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.control.kinematics import Kinematics

# SCARA DOF
_DOF = 3

# Tolerances
_POS_TOL_M = 0.005  # 5 mm IK round-trip budget
_ANG_TOL_RAD = 1e-9

# Expected XACRO constants
_L1 = 0.325
_L2 = 0.275
_Z_BASE = 0.1665 + 0.0715  # 0.238


def test_construction() -> None:
    Kinematics(model="scara")


def test_unsupported_model_raises() -> None:
    with pytest.raises(ValueError, match="Unsupported model"):
        Kinematics(model="delta")


def test_dof_is_three() -> None:
    k = Kinematics(model="scara")
    assert k.dof == _DOF


def test_joint_names_length() -> None:
    k = Kinematics(model="scara")
    names = k.joint_names
    assert len(names) == _DOF


def test_joint_names_are_xacro_names() -> None:
    k = Kinematics(model="scara")
    assert k.joint_names == [
        "joint_arm_0",
        "joint_arm_1",
        "joint_extension",
    ]


def test_joint_limits_shape() -> None:
    k = Kinematics(model="scara")
    limits = k.joint_limits
    assert limits.shape == (
        _DOF,
        2,
    ), f"Expected ({_DOF}, 2), got {limits.shape}"
    # lower < upper for every joint
    assert np.all(limits[:, 0] < limits[:, 1])


def test_joint_limits_values() -> None:
    """Verify the XACRO-sourced limit values are loaded correctly."""
    k = Kinematics(model="scara")
    lim = k.joint_limits
    assert abs(lim[0, 0] - (-132.0 * math.pi / 180.0)) < _ANG_TOL_RAD
    assert abs(lim[0, 1] - (132.0 * math.pi / 180.0)) < _ANG_TOL_RAD
    assert abs(lim[1, 0] - (-150.0 * math.pi / 180.0)) < _ANG_TOL_RAD
    assert abs(lim[1, 1] - (150.0 * math.pi / 180.0)) < _ANG_TOL_RAD
    assert lim[2, 0] == pytest.approx(0.0)
    assert lim[2, 1] == pytest.approx(0.2)


def test_forward_kinematics_home_returns_4x4() -> None:
    k = Kinematics(model="scara")
    T = k.forward_kinematics(np.zeros(_DOF))
    assert T.shape == (4, 4), f"Expected (4, 4), got {T.shape}"
    # Last row must be [0, 0, 0, 1]
    np.testing.assert_array_almost_equal(T[3, :], [0.0, 0.0, 0.0, 1.0])


def test_forward_kinematics_home_position() -> None:
    """At q = [0, 0, 0] the EE should be at (L1+L2, 0, z_base)."""
    k = Kinematics(model="scara")
    T = k.forward_kinematics(np.zeros(_DOF))
    assert T[0, 3] == pytest.approx(_L1 + _L2, abs=1e-9)
    assert T[1, 3] == pytest.approx(0.0, abs=1e-9)
    assert T[2, 3] == pytest.approx(_Z_BASE, abs=1e-9)


def test_forward_kinematics_identity_rotation_at_home() -> None:
    """At q1=0, q2=0 the rotation part is identity (R = R_z(0))."""
    k = Kinematics(model="scara")
    T = k.forward_kinematics(np.zeros(_DOF))
    np.testing.assert_array_almost_equal(T[:3, :3], np.eye(3))


def test_forward_kinematics_z_decreases_with_q3() -> None:
    """Increasing q3 should lower the EE (z_ee = z_base - q3)."""
    k = Kinematics(model="scara")
    T0 = k.forward_kinematics(np.array([0.0, 0.0, 0.0]))
    T1 = k.forward_kinematics(np.array([0.0, 0.0, 0.1]))
    assert T1[2, 3] == pytest.approx(T0[2, 3] - 0.1, abs=1e-9)


def test_jacobian_shape() -> None:
    k = Kinematics(model="scara")
    J = k.jacobian(np.zeros(_DOF))
    assert J.shape == (6, _DOF), f"Expected (6, {_DOF}), got {J.shape}"


def test_jacobian_at_home_linear_z_column() -> None:
    """The prismatic joint column must be [0, 0, -1, 0, 0, 0]."""
    k = Kinematics(model="scara")
    J = k.jacobian(np.zeros(_DOF))
    np.testing.assert_array_almost_equal(
        J[:, 2], [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
    )


def test_jacobian_angular_rows_at_home() -> None:
    """Revolute joints have angular component [0, 0, 1]; prismatic is zero."""
    k = Kinematics(model="scara")
    J = k.jacobian(np.zeros(_DOF))
    # ω contributions for joint 1 and 2 are [0, 0, 1]
    np.testing.assert_array_almost_equal(J[3:, 0], [0.0, 0.0, 1.0])
    np.testing.assert_array_almost_equal(J[3:, 1], [0.0, 0.0, 1.0])
    # prismatic: no angular contribution
    np.testing.assert_array_almost_equal(J[3:, 2], [0.0, 0.0, 0.0])


def test_ik_round_trip() -> None:
    """FK(IK(target)) ≈ target within 5 mm EE error."""
    k = Kinematics(model="scara")
    q_ref = np.array([0.3, 0.2, 0.05])
    T_target = k.forward_kinematics(q_ref)
    q_ik = k.inverse_kinematics(T_target, seed=np.zeros(_DOF))
    T_recovered = k.forward_kinematics(q_ik)
    position_error = np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])
    assert (
        position_error < _POS_TOL_M
    ), f"EE error {position_error:.4f} m exceeds 5 mm"


def test_ik_multiple_configurations() -> None:
    """IK round-trip holds for a range of reachable configurations."""
    k = Kinematics(model="scara")
    configs = [
        [0.0, 0.0, 0.0],
        [1.0, -0.5, 0.1],
        [-0.8, 1.2, 0.05],
        [0.5, 0.5, 0.2],
    ]
    for q_ref_list in configs:
        q_ref = np.array(q_ref_list)
        T_target = k.forward_kinematics(q_ref)
        q_ik = k.inverse_kinematics(T_target)
        T_recovered = k.forward_kinematics(q_ik)
        err = np.linalg.norm(T_recovered[:3, 3] - T_target[:3, 3])
        assert err < _POS_TOL_M, f"q={q_ref} → error {err:.4f} m"


def test_ik_out_of_reach_raises() -> None:
    """IK must raise RuntimeError for an unreachable target."""
    k = Kinematics(model="scara")
    # Place target far outside reach (L1+L2 = 0.6 m)
    pose = np.eye(4)
    pose[0, 3] = 2.0  # 2 m along x — unreachable
    with pytest.raises(RuntimeError):
        k.inverse_kinematics(pose)


def test_forward_kinematics_wrong_shape_raises() -> None:
    k = Kinematics(model="scara")
    with pytest.raises(ValueError):
        k.forward_kinematics(np.zeros(5))  # wrong DOF


def test_jacobian_wrong_shape_raises() -> None:
    k = Kinematics(model="scara")
    with pytest.raises(ValueError):
        k.jacobian(np.zeros(5))
