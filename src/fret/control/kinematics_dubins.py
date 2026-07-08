"""SE(2) kinematics facade for the Dubins mobile robot (T11-01).

State ``q = (x, y, θ)`` in the warehouse floor plane.  Execution uses
ARCO ``DubinsVehicle`` directly; this module satisfies ``Kinematics`` /
``CSpaceChecker`` integration for the FRET planning stack.
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

_DEFAULT_BOUNDS: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 24.0],
        [0.0, 16.0],
        [-math.pi, math.pi],
    ],
    dtype=np.float64,
)

_JOINT_NAMES: list[str] = ["joint_x", "joint_y", "joint_yaw"]


class DubinsKinematics:
    """Minimal SE(2) kinematics engine for the Dubins race scenario."""

    def __init__(
        self,
        limits: npt.NDArray[np.float64] | None = None,
    ) -> None:
        self._limits = (
            _DEFAULT_BOUNDS.copy()
            if limits is None
            else np.asarray(limits, dtype=np.float64)
        )

    @property
    def dof(self) -> int:
        """Number of configuration coordinates (3)."""
        return 3

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names for MuJoCo / ROS I/O."""
        return list(_JOINT_NAMES)

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(3, 2)``."""
        return self._limits.copy()

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return a 4×4 pose matrix for the chassis frame."""
        q = np.asarray(joint_positions, dtype=np.float64)
        if q.shape != (3,):
            raise ValueError(f"Expected shape (3,), got {q.shape}")
        x, y, theta = float(q[0]), float(q[1]), float(q[2])
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array(
            [
                [c, -s, 0.0, x],
                [s, c, 0.0, y],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Extract ``(x, y, θ)`` from a 4×4 pose matrix."""
        del seed
        pose = np.asarray(ee_pose, dtype=np.float64)
        if pose.shape != (4, 4):
            raise ValueError(f"Expected pose shape (4, 4), got {pose.shape}")
        theta = math.atan2(float(pose[1, 0]), float(pose[0, 0]))
        return np.array(
            [float(pose[0, 3]), float(pose[1, 3]), theta],
            dtype=np.float64,
        )

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return identity Jacobian for holonomic SE(2) queries."""
        del joint_positions
        return np.eye(3, dtype=np.float64)

    @staticmethod
    def position_from_config(
        config: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Return planar position ``(x, y)`` from a configuration."""
        q = np.asarray(config, dtype=np.float64)
        return q[:2].copy()
