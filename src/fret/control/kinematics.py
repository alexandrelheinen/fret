"""Per-model kinematics facade for FRET robot models.

Dispatches to model-specific engines:

- ``"scara"`` — SCARA RRP (bootstrap, v0.9)
- ``"ppp"`` — PPP gantry (v1.0)

Satisfies requirements FR-SYS-01, FR-CTL-02, and FR-PLN-02.
"""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np
import numpy.typing as npt

from fret.control.kinematics_ppp import PPPKinematics

# ---------------------------------------------------------------------------
# SCARA model constants (source: src/fret/urdf/scara.xacro)
# ---------------------------------------------------------------------------

#: Kinematic length of link 1 (``arm_0_length``) [m].
_L1: float = 0.325
#: Kinematic length of link 2 (``arm_1_length``) [m].
_L2: float = 0.275
#: Height of joint J1 above base origin (``link_0_cylinder_height``) [m].
_J1_Z: float = 0.1665
#: Vertical offset from arm_0_link to joint J2 (``link_1_height``) [m].
_J2_Z: float = 0.0715
#: EE height when q3 = 0 (j1_z + j2_z) [m].
_Z_BASE: float = _J1_Z + _J2_Z

_J1_LOWER: float = -132.0 * math.pi / 180.0
_J1_UPPER: float = 132.0 * math.pi / 180.0
_J2_LOWER: float = -150.0 * math.pi / 180.0
_J2_UPPER: float = 150.0 * math.pi / 180.0
_J3_LOWER: float = 0.0
_J3_UPPER: float = 0.2

#: URDF joint names in kinematic-chain order.
_JOINT_NAMES: list[str] = [
    "joint_arm_0",
    "joint_arm_1",
    "joint_extension",
]


class _KinematicsBackend(Protocol):
    """Structural protocol shared by per-model kinematics engines."""

    @property
    def dof(self) -> int: ...

    @property
    def joint_names(self) -> list[str]: ...

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]: ...

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]: ...

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]: ...

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]: ...


class _ScaraKinematics:
    """Closed-form FK, IK, and Jacobian for the FRET SCARA RRP model."""

    def __init__(self) -> None:
        self._l1: float = _L1
        self._l2: float = _L2
        self._z_base: float = _Z_BASE
        self._limits: npt.NDArray[np.float64] = np.array(
            [
                [_J1_LOWER, _J1_UPPER],
                [_J2_LOWER, _J2_UPPER],
                [_J3_LOWER, _J3_UPPER],
            ],
            dtype=np.float64,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def dof(self) -> int:
        """Number of degrees of freedom (3 for the SCARA RRP model)."""
        return 3

    @property
    def joint_names(self) -> list[str]:
        """URDF joint names in kinematic-chain order (length ``DOF``)."""
        return list(_JOINT_NAMES)

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)`` — columns: lower, upper.

        Units: radians for revolute joints, metres for the prismatic joint.
        """
        return self._limits.copy()

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the homogeneous end-effector pose from joint positions.

        The returned transform maps points in the end-effector frame to the
        ``world`` frame, consistent with the SCARA XACRO kinematic chain.

        Kinematic equations (``c1 = cos(q1)``, ``c12 = cos(q1+q2)``):

        .. code-block:: text

            x_ee = L1·cos(q1) + L2·cos(q1+q2)
            y_ee = L1·sin(q1) + L2·sin(q1+q2)
            z_ee = z_base − q3

        Args:
            joint_positions: Joint configuration, shape ``(DOF,)``.

        Returns:
            4×4 homogeneous transformation matrix (dtype ``float64``).

        Raises:
            ValueError: If ``joint_positions.shape != (DOF,)``.
        """
        if joint_positions.shape != (self.dof,):
            raise ValueError(
                f"Expected shape ({self.dof},), got {joint_positions.shape}"
            )
        q1 = float(joint_positions[0])
        q2 = float(joint_positions[1])
        q3 = float(joint_positions[2])

        c1, s1 = math.cos(q1), math.sin(q1)
        c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)

        x = self._l1 * c1 + self._l2 * c12
        y = self._l1 * s1 + self._l2 * s12
        z = self._z_base - q3

        return np.array(
            [
                [c12, -s12, 0.0, x],
                [s12, c12, 0.0, y],
                [0.0, 0.0, 1.0, z],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Compute a joint configuration that achieves the given EE pose.

        The SCARA IK is closed-form and position-only: the EE orientation is
        uniquely determined by ``q1 + q2`` and is always consistent with the FK
        output.  Elbow-up is tried first; elbow-down is the fallback.

        The ``seed`` argument is accepted for API compatibility with generic
        solvers but is not used by this closed-form implementation.

        Args:
            ee_pose: 4×4 homogeneous pose matrix in the ``world`` frame.
            seed: Optional initial guess, shape ``(DOF,)``.  Unused.

        Returns:
            Joint configuration, shape ``(DOF,)``.

        Raises:
            RuntimeError: If no solution exists within joint limits.
        """
        x_t = float(ee_pose[0, 3])
        y_t = float(ee_pose[1, 3])
        z_t = float(ee_pose[2, 3])

        # Prismatic joint (extends in –z direction)
        q3_raw = self._z_base - z_t
        if not (_J3_LOWER - 1e-9 <= q3_raw <= _J3_UPPER + 1e-9):
            raise RuntimeError(
                f"Target z={z_t:.4f} m implies q3={q3_raw:.4f} m outside "
                f"limits [{_J3_LOWER}, {_J3_UPPER}]"
            )
        q3 = float(np.clip(q3_raw, _J3_LOWER, _J3_UPPER))

        # Two-link planar IK (cosine rule)
        r2 = x_t**2 + y_t**2
        c2_raw = (r2 - self._l1**2 - self._l2**2) / (2.0 * self._l1 * self._l2)
        if abs(c2_raw) > 1.0 + 1e-9:
            raise RuntimeError(
                f"Target (x={x_t:.4f} m, y={y_t:.4f} m) is out of reach: "
                f"cos(q2) = {c2_raw:.4f}"
            )
        c2 = float(np.clip(c2_raw, -1.0, 1.0))

        for sign in (1.0, -1.0):  # elbow-up first, then elbow-down
            s2 = sign * math.sqrt(max(0.0, 1.0 - c2**2))
            q2 = math.atan2(s2, c2)
            k1 = self._l1 + self._l2 * c2
            k2 = self._l2 * s2
            q1 = math.atan2(y_t, x_t) - math.atan2(k2, k1)
            if (
                _J1_LOWER - 1e-9 <= q1 <= _J1_UPPER + 1e-9
                and _J2_LOWER - 1e-9 <= q2 <= _J2_UPPER + 1e-9
            ):
                return np.array(
                    [
                        float(np.clip(q1, _J1_LOWER, _J1_UPPER)),
                        float(np.clip(q2, _J2_LOWER, _J2_UPPER)),
                        q3,
                    ],
                    dtype=np.float64,
                )

        raise RuntimeError(
            f"No IK solution within joint limits for target "
            f"(x={x_t:.4f} m, y={y_t:.4f} m, z={z_t:.4f} m)"
        )

    # ------------------------------------------------------------------
    # Jacobian
    # ------------------------------------------------------------------

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return the geometric Jacobian at the given configuration.

        The Jacobian maps joint velocities to the 6-D end-effector twist
        ``[v_x, v_y, v_z, ω_x, ω_y, ω_z]`` expressed in the ``world`` frame.

        Columns (joints 1–3):

        .. code-block:: text

            J = [−(L1·s1 + L2·s12),  −L2·s12,   0 ]
                [  L1·c1 + L2·c12,    L2·c12,   0 ]
                [        0,                0,  −1 ]
                [        0,                0,   0 ]
                [        0,                0,   0 ]
                [        1,                1,   0 ]

        Args:
            joint_positions: Joint configuration, shape ``(DOF,)``.

        Returns:
            Geometric Jacobian, shape ``(6, DOF)``, dtype ``float64``.

        Raises:
            ValueError: If ``joint_positions.shape != (DOF,)``.
        """
        if joint_positions.shape != (self.dof,):
            raise ValueError(
                f"Expected shape ({self.dof},), got {joint_positions.shape}"
            )
        q1 = float(joint_positions[0])
        q2 = float(joint_positions[1])

        c1, s1 = math.cos(q1), math.sin(q1)
        c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)

        jv1x = -(self._l1 * s1 + self._l2 * s12)
        jv1y = self._l1 * c1 + self._l2 * c12
        jv2x = -self._l2 * s12
        jv2y = self._l2 * c12

        return np.array(
            [
                [jv1x, jv2x, 0.0],
                [jv1y, jv2y, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )


_SUPPORTED_MODELS: frozenset[str] = frozenset({"scara", "ppp"})


class Kinematics:
    """Facade for per-model kinematics engines (FR-SYS-01).

    A single ``Kinematics`` instance is shared between ``CSpaceChecker``
    (planning layer) and ``StateEstimator`` (control layer) to avoid
    redundant computation.

    Args:
        model: Robot model name — ``"scara"`` or ``"ppp"``.

    Raises:
        ValueError: If ``model`` is not supported.
    """

    def __init__(self, model: str) -> None:
        if model not in _SUPPORTED_MODELS:
            supported = ", ".join(sorted(_SUPPORTED_MODELS))
            raise ValueError(
                f"Unsupported model '{model}'. Supported models: {supported}"
            )
        if model == "scara":
            self._impl: _KinematicsBackend = _ScaraKinematics()
        else:
            self._impl = PPPKinematics()

    @property
    def dof(self) -> int:
        """Number of degrees of freedom for the active model."""
        return self._impl.dof

    @property
    def joint_names(self) -> list[str]:
        """URDF joint names in kinematic-chain order (length ``DOF``)."""
        return self._impl.joint_names

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)`` — columns: lower, upper."""
        return self._impl.joint_limits

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the homogeneous end-effector pose from joint positions."""
        return self._impl.forward_kinematics(joint_positions)

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Compute a joint configuration that achieves the given EE pose."""
        return self._impl.inverse_kinematics(ee_pose, seed=seed)

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return the geometric Jacobian at the given configuration."""
        return self._impl.jacobian(joint_positions)
