"""Per-model kinematics facade for FRET robot models.

Dispatches to model-specific engines:

- ``"dubins"`` — Dubins mobile (v1.1)

Arm models (OpenMANIPULATOR-X, …) register here as they land.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
import numpy.typing as npt

from fret.control.kinematics_dubins import DubinsKinematics


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


_SUPPORTED_MODELS: frozenset[str] = frozenset({"dubins"})


class Kinematics:
    """Facade for per-model kinematics engines (FR-SYS-01).

    Args:
        model: Robot model name — currently ``"dubins"``.

    Raises:
        ValueError: If ``model`` is not supported.
    """

    def __init__(self, model: str) -> None:
        if model not in _SUPPORTED_MODELS:
            supported = ", ".join(sorted(_SUPPORTED_MODELS))
            raise ValueError(
                f"Unsupported model '{model}'. Supported models: {supported}"
            )
        self._impl: _KinematicsBackend = DubinsKinematics()

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
