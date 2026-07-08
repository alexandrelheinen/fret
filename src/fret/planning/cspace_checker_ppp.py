"""PPP C-space collision checker: full gantry envelope + optional cargo (T10-05).

For the PPP gantry, configuration space is Cartesian position
``q ≡ p_ee``.  Collision checking samples axis-aligned bounding boxes
for all moving MJCF bodies (Z column, gripper, overhead bridge) and
optionally the welded cargo box.

Satisfies requirements FR-PLN-02 and FR-GSP-02.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from fret.control.grasp_magnet import GraspConfig
from fret.planning.cspace_checker import occupancy_min_clearance
from fret.planning.ppp_robot_envelope import (
    BodyEnvelope,
    ppp_envelope_sample_points,
)

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics


@dataclass
class PPPCheckerConfig:
    """Geometry for PPP collision envelope sampling.

    Attributes:
        include_cargo: When True, sample welded cargo box (FR-GSP-02).
        grasp_config: Cargo geometry and weld offset from grasp module.
        workspace_bounds: Optional ``(x, y, z)`` min/max pairs [m].  When
            set, configurations outside the box are treated as colliding.
    """

    include_cargo: bool = False
    grasp_config: GraspConfig = field(default_factory=GraspConfig)
    workspace_bounds: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ] | None = None


class PPPcSpaceChecker:
    """C-space checker for PPP with multi-body gantry envelopes.

    Args:
        kinematics: ``Kinematics(model="ppp")`` instance.
        occupancy: Occupancy model with ``clearance`` or ``query_distances``.
        config: Cargo sampling configuration.
    """

    def __init__(
        self,
        kinematics: Kinematics,
        occupancy: Any,
        config: PPPCheckerConfig | None = None,
    ) -> None:
        self._kin = kinematics
        self._occ = occupancy
        self._config = config if config is not None else PPPCheckerConfig()
        self._dof: int = kinematics.dof

    @property
    def include_cargo(self) -> bool:
        """Whether welded cargo is included in collision checks."""
        return self._config.include_cargo

    def _cargo_envelope(
        self, configuration: npt.NDArray[np.float64]
    ) -> BodyEnvelope | None:
        if not self._config.include_cargo:
            return None
        ee_pos = configuration.astype(np.float64)
        grasp = self._config.grasp_config
        return BodyEnvelope(
            centre=ee_pos + grasp.weld_offset,
            half_extent=grasp.box_half_extent.copy(),
        )

    def _sample_check_points(
        self, configuration: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return world-frame sample points for gantry and optional cargo."""
        if configuration.shape != (self._dof,):
            raise ValueError(
                f"Expected shape ({self._dof},), got {configuration.shape}"
            )
        cargo = self._cargo_envelope(configuration)
        extras = [cargo] if cargo is not None else None
        return ppp_envelope_sample_points(configuration, extra_envelopes=extras)

    def _within_joint_limits(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        limits = self._kin.joint_limits
        if not (
            np.all(configuration >= limits[:, 0] - 1e-9)
            and np.all(configuration <= limits[:, 1] + 1e-9)
        ):
            return False
        bounds = self._config.workspace_bounds
        if bounds is None:
            return True
        (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi) = bounds
        x, y, z = float(configuration[0]), float(configuration[1]), float(configuration[2])
        return (
            x_lo - 1e-9 <= x <= x_hi + 1e-9
            and y_lo - 1e-9 <= y <= y_hi + 1e-9
            and z_lo - 1e-9 <= z <= z_hi + 1e-9
        )

    def is_collision_free(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        """Return True if gantry bodies (and cargo if enabled) are free."""
        return self.clearance(configuration) > 0.0

    def clearance(self, configuration: npt.NDArray[np.float64]) -> float:
        """Minimum clearance over all gantry/cargo sample points [m]."""
        if configuration.shape != (self._dof,):
            raise ValueError(
                f"Expected shape ({self._dof},), got {configuration.shape}"
            )
        if not self._within_joint_limits(configuration):
            return -1.0
        pts = self._sample_check_points(configuration)
        return occupancy_min_clearance(self._occ, pts)
