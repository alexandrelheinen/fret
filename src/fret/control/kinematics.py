"""Kinematics engine for a named FRET robot model.

Provides forward kinematics (FK), inverse kinematics (IK), and the
geometric Jacobian for a parametric robot description loaded from a
URDF/XACRO file at construction time.

The SCARA RRP model (3 DOF: joint_1 [−π, π], joint_2 [−π/2, π/2],
joint_3 [0, 0.2 m]) is the primary target.  The interface is generic
enough to support the delta target (Phase 4).

Satisfies requirements FR-CTL-02 and FR-PLN-02.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class Kinematics:
    """FK, IK, and Jacobian computation for a named FRET robot model.

    A single ``Kinematics`` instance is shared between ``PlannerNode``
    (via ``CSpaceChecker``) and ``ControllerNode`` (via ``StateEstimator``)
    to avoid redundant URDF parsing.

    Args:
        model: Name of the robot model (e.g. ``"scara"``).  The corresponding
            XACRO file must be resolvable via ``ament_index``.
    """

    def __init__(self, model: str) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def dof(self) -> int:
        """Number of degrees of freedom of the loaded model."""
        raise NotImplementedError

    @property
    def joint_names(self) -> list[str]:
        """URDF joint names in kinematic chain order."""
        raise NotImplementedError

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)`` — columns: [lower, upper].

        Units: radians for revolute joints, metres for prismatic joints.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Core computations
    # ------------------------------------------------------------------

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the end-effector pose from a joint configuration.

        Args:
            joint_positions: Joint configuration, shape ``(DOF,)``.

        Returns:
            End-effector pose as a ``(4, 4)`` homogeneous transformation
            matrix expressed in the ``world`` frame.

        Raises:
            ValueError: If ``joint_positions.shape != (DOF,)``.
        """
        raise NotImplementedError

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Compute a joint configuration that achieves the given EE pose.

        Args:
            ee_pose: Target end-effector pose as a ``(4, 4)`` homogeneous
                transformation matrix.
            seed: Optional initial guess for the solver, shape ``(DOF,)``.
                Defaults to the zero configuration.

        Returns:
            Joint configuration, shape ``(DOF,)``, satisfying
            ``FK(q) ≈ ee_pose`` within the EE tracking-error budget of 5 mm.

        Raises:
            RuntimeError: If the solver fails to converge.
        """
        raise NotImplementedError

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the geometric Jacobian at the given configuration.

        Args:
            joint_positions: Joint configuration, shape ``(DOF,)``.

        Returns:
            Geometric Jacobian, shape ``(6, DOF)``.  Rows 0–2 are the
            linear-velocity part; rows 3–5 are the angular-velocity part.

        Raises:
            ValueError: If ``joint_positions.shape != (DOF,)``.
        """
        raise NotImplementedError
