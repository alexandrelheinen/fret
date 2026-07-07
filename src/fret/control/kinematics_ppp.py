"""Kinematics engine for the FRET PPP gantry robot model (v1.0).

The PPP gantry has three prismatic joints (X, Y, Z).  Configuration space is
identical to Cartesian workspace position: ``q = (x, y, z)`` in metres.

Parameters are defined in ``docs/robots/ppp.md``.

Satisfies requirements FR-SYS-01, FR-CTL-02, and FR-PLN-02.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

#: Joint limits [lower, upper] in metres (source: docs/robots/ppp.md).
_X_LIMITS: tuple[float, float] = (0.0, 60.0)
_Y_LIMITS: tuple[float, float] = (0.0, 20.0)
_Z_LIMITS: tuple[float, float] = (0.0, 6.0)

#: URDF joint names in kinematic-chain order.
_JOINT_NAMES: list[str] = ["joint_x", "joint_y", "joint_z"]


class PPPKinematics:
    """Identity-map FK, IK, and Jacobian for the PPP gantry model.

    Forward kinematics maps joint configuration directly to end-effector
    position in the ``world`` frame.  Inverse kinematics is the inverse map
    with joint-limit enforcement.

    A ``PPPKinematics`` instance is interchangeable with ``Kinematics(model="ppp")``.
    """

    def __init__(self) -> None:
        self._limits: npt.NDArray[np.float64] = np.array(
            [_X_LIMITS, _Y_LIMITS, _Z_LIMITS],
            dtype=np.float64,
        )

    @property
    def dof(self) -> int:
        """Number of degrees of freedom (3 for the PPP gantry)."""
        return 3

    @property
    def joint_names(self) -> list[str]:
        """URDF joint names in kinematic-chain order (length ``DOF``)."""
        return list(_JOINT_NAMES)

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)`` — columns: lower, upper [m]."""
        return self._limits.copy()

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the homogeneous end-effector pose from joint positions.

        The PPP FK is the identity map: ``p_ee = q``.

        Args:
            joint_positions: Joint configuration, shape ``(DOF,)`` [m].

        Returns:
            4×4 homogeneous transformation matrix (dtype ``float64``).

        Raises:
            ValueError: If ``joint_positions.shape != (DOF,)``.
        """
        if joint_positions.shape != (self.dof,):
            raise ValueError(
                f"Expected shape ({self.dof},), got {joint_positions.shape}"
            )
        x = float(joint_positions[0])
        y = float(joint_positions[1])
        z = float(joint_positions[2])
        return np.array(
            [
                [1.0, 0.0, 0.0, x],
                [0.0, 1.0, 0.0, y],
                [0.0, 0.0, 1.0, z],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Compute the joint configuration for the given EE pose.

        The PPP IK is the inverse identity map: ``q = p_ee``.  Targets outside
        the operational envelope raise ``RuntimeError``.

        Args:
            ee_pose: 4×4 homogeneous pose matrix in the ``world`` frame.
            seed: Optional initial guess, shape ``(DOF,)``.  Unused.

        Returns:
            Joint configuration, shape ``(DOF,)`` [m].

        Raises:
            RuntimeError: If the target position is outside joint limits.
        """
        del seed  # unused — closed-form IK
        x_t = float(ee_pose[0, 3])
        y_t = float(ee_pose[1, 3])
        z_t = float(ee_pose[2, 3])
        limits = self._limits
        coords = (x_t, y_t, z_t)
        for idx, value in enumerate(coords):
            lower = float(limits[idx, 0])
            upper = float(limits[idx, 1])
            if not (lower - 1e-9 <= value <= upper + 1e-9):
                raise RuntimeError(
                    f"Target axis {idx}={value:.4f} m outside limits "
                    f"[{lower}, {upper}]"
                )
        return np.array([x_t, y_t, z_t], dtype=np.float64)

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return the geometric Jacobian at the given configuration.

        Prismatic joints aligned with world X, Y, Z produce a constant
        identity linear block with zero angular rows.

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
        # Jacobian is configuration-independent for PPP prismatic axes.
        return np.array(
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
