"""PPP C-space collision checker: EE + optional cargo envelope (T10-05).

For the PPP gantry, configuration space is Cartesian position
``q ≡ p_ee``.  Collision checking samples axis-aligned bounding boxes
around the end-effector and (when enabled) the welded cargo box.

Satisfies requirements FR-PLN-02 and FR-GSP-02.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from fret.control.grasp_magnet import GraspConfig
from fret.planning.cspace_checker import occupancy_min_clearance

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics


_DEFAULT_EE_HALF: npt.NDArray[np.float64] = np.array(
    [0.25, 0.25, 0.04], dtype=np.float64
)


@dataclass
class PPPCheckerConfig:
    """Geometry for PPP collision envelope sampling.

    Attributes:
        ee_half_extent: EE AABB half-sizes [m] (0.5 m × 0.5 m face).
        include_cargo: When True, sample welded cargo box (FR-GSP-02).
        grasp_config: Cargo geometry and weld offset from grasp module.
    """

    ee_half_extent: npt.NDArray[np.float64] = field(
        default_factory=lambda: _DEFAULT_EE_HALF.copy()
    )
    include_cargo: bool = False
    grasp_config: GraspConfig = field(default_factory=GraspConfig)


class PPPcSpaceChecker:
    """C-space checker for PPP with EE and optional cargo envelopes.

    Args:
        kinematics: ``Kinematics(model="ppp")`` instance.
        occupancy: Occupancy model with ``clearance`` or ``query_distances``.
        config: EE/cargo sampling configuration.
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

    def _box_corners(
        self,
        centre: npt.NDArray[np.float64],
        half_extent: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        hx, hy, hz = half_extent
        offsets = np.array(
            [
                [-hx, -hy, -hz],
                [hx, -hy, -hz],
                [-hx, hy, -hz],
                [hx, hy, -hz],
                [-hx, -hy, hz],
                [hx, -hy, hz],
                [-hx, hy, hz],
                [hx, hy, hz],
            ],
            dtype=np.float64,
        )
        return centre + offsets

    def _sample_check_points(
        self, configuration: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return world-frame sample points for EE and optional cargo AABBs."""
        if configuration.shape != (self._dof,):
            raise ValueError(
                f"Expected shape ({self._dof},), got {configuration.shape}"
            )
        ee_pos = self._kin.forward_kinematics(configuration)[:3, 3]
        cfg = self._config
        samples = [self._box_corners(ee_pos, cfg.ee_half_extent)]

        if cfg.include_cargo:
            grasp = cfg.grasp_config
            cargo_centre = ee_pos + grasp.weld_offset
            samples.append(
                self._box_corners(cargo_centre, grasp.box_half_extent)
            )

        return np.vstack(samples)

    def is_collision_free(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        """Return True if EE (and cargo if enabled) are collision-free."""
        return self.clearance(configuration) > 0.0

    def clearance(self, configuration: npt.NDArray[np.float64]) -> float:
        """Minimum clearance over all EE/cargo sample points [m]."""
        pts = self._sample_check_points(configuration)
        return occupancy_min_clearance(self._occ, pts)
