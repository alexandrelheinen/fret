"""Per-model kinematics facade for FRET robot models (FR-SYS-01, FR-SYS-08).

Thin facade over :mod:`fret.robots.registry` so existing call sites and
tests keep compiling unchanged (``control/kinematics.py::Kinematics(model)``
design note in ``docs/robot-model-agnostic.md``). The actual per-model
dispatch lives in the registry; this module contains no model-name string
literals.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from fret.robots.protocols import KinematicsBackend
from fret.robots.registry import get_default_registry


class Kinematics:
    """Facade for per-model kinematics engines (FR-SYS-01).

    Args:
        model: Robot model name or alias, resolved through the robot
            registry (``config/robots/*.yml``).

    Raises:
        fret.robots.registry.UnknownRobotModelError: If ``model`` is not a
            known key or alias.
    """

    def __init__(self, model: str) -> None:
        self._impl: KinematicsBackend = get_default_registry().get_kinematics(
            model
        )

    @property
    def dof(self) -> int:
        """Number of degrees of freedom for the active model."""
        return self._impl.dof

    @property
    def joint_names(self) -> list[str]:
        """Joint names in kinematic-chain order (length ``DOF``)."""
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
