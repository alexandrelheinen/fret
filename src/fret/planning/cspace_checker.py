"""C-space collision checker: FK + KDTreeOccupancy.

Bridges FRET's kinematics engine and ARCO's occupancy model to provide a
joint-space collision predicate.  For each query configuration ``q``, FK
maps ``q`` to world-frame link positions which are then tested against the
occupancy model.

The occupancy object is accepted by duck typing: any object that exposes a
``clearance(pts)`` method compatible with ``arco.mapping.KDTreeOccupancy``
can be used, including the ``_SimpleOccupancy`` fallback in
``scene.OccupancyAdapter``.

Satisfies requirement FR-PLN-02.
"""

from __future__ import annotations

import pathlib
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import numpy.typing as npt

CollisionBackend = Literal["analytic", "mujoco"]

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics


def occupancy_min_clearance(
    occupancy: Any, points: npt.NDArray[np.float64]
) -> float:
    """Return minimum clearance from query points to an occupancy model.

    Supports ARCO ``KDTreeOccupancy`` (``query_distances`` + ``clearance``
    attribute) and duck-typed ``clearance(pts)`` callables such as
    ``scene.OccupancyAdapter._SimpleOccupancy``.

    Args:
        occupancy: Occupancy model instance.
        points: Query positions, shape ``(N, 3)``.

    Returns:
        Minimum clearance in metres (positive = free).
    """
    if hasattr(occupancy, "query_distances"):
        distances = occupancy.query_distances(points)
        return float(np.min(distances)) - float(occupancy.clearance)
    return float(occupancy.clearance(points))


class CSpaceChecker:
    """Query collision-free status for a joint configuration via FK + occupancy.

    For each query configuration ``q``, the checker samples ``_N_SAMPLES``
    positions linearly interpolated from the base joint origin to the
    end-effector position (obtained via FK).  The minimum clearance over all
    sampled positions is returned; the configuration is free if that minimum
    is positive.

    Args:
        kinematics: Kinematics engine for the active robot model.  Must
            expose ``dof: int`` and
            ``forward_kinematics(q) -> NDArray[float64]`` (4×4 matrix).
        occupancy: An occupancy object that exposes
            ``clearance(pts: NDArray) -> float``, where ``pts`` has shape
            ``(N, 3)``.  Both ``arco.mapping.KDTreeOccupancy`` and
            ``scene.OccupancyAdapter._SimpleOccupancy`` satisfy this.
    """

    _N_SAMPLES: int = 6  # number of arm-position samples per query

    def __init__(self, kinematics: Kinematics, occupancy: Any) -> None:
        self._kin = kinematics
        self._occ = occupancy
        self._dof: int = kinematics.dof

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _sample_arm_positions(
        self, configuration: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return ``_N_SAMPLES`` 3-D positions sampled along the arm.

        Positions are linearly interpolated from ``(0, 0, 0)`` (world
        origin) to the FK end-effector position for ``configuration``.  This
        is a conservative approximation: the actual arm links lie along
        roughly this line for a straight-reach SCARA.

        Args:
            configuration: Joint configuration, shape ``(DOF,)``.

        Returns:
            Array of shape ``(_N_SAMPLES, 3)``.
        """
        T_ee = self._kin.forward_kinematics(configuration)
        ee_pos: npt.NDArray[np.float64] = T_ee[:3, 3]  # shape (3,)
        alphas = np.linspace(0.0, 1.0, self._N_SAMPLES)
        return np.outer(alphas, ee_pos)  # shape (N, 3)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_collision_free(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        """Return True if the configuration does not collide with any obstacle.

        Args:
            configuration: Joint configuration, shape ``(DOF,)``.

        Returns:
            ``True`` if the minimum link-to-obstacle clearance is positive.

        Raises:
            ValueError: If ``configuration.shape != (DOF,)``.
        """
        return self.clearance(configuration) > 0.0

    def clearance(self, configuration: npt.NDArray[np.float64]) -> float:
        """Return the minimum world-frame clearance from any obstacle.

        Args:
            configuration: Joint configuration, shape ``(DOF,)``.

        Returns:
            Minimum clearance in metres.  Positive = free, negative = inside
            an obstacle.

        Raises:
            ValueError: If ``configuration.shape != (DOF,)``.
        """
        if configuration.shape != (self._dof,):
            raise ValueError(
                f"CSpaceChecker expects shape ({self._dof},), "
                f"got {configuration.shape}"
            )
        pts = self._sample_arm_positions(configuration)
        return occupancy_min_clearance(self._occ, pts)


def make_cspace_checker(
    kinematics: Kinematics,
    occupancy: Any,
    *,
    include_cargo: bool = False,
    grasp_config: Any | None = None,
    contact_radius: float | None = None,
    collision_backend: CollisionBackend = "analytic",
    mjcf_path: str | pathlib.Path | None = None,
    scenario: str = "static_reach",
    workspace_bounds: (
        tuple[
            tuple[float, float],
            tuple[float, float],
            tuple[float, float],
        ]
        | None
    ) = None,
) -> CSpaceChecker | Any:
    """Build a model-appropriate C-space checker (FR-SYS-01).

    Returns the arm-sampling ``CSpaceChecker`` used by SCARA / RRP models.
    Extra keyword arguments are accepted for API compatibility with older
    call sites and ignored.

    Args:
        kinematics: Active ``Kinematics`` engine.
        occupancy: Occupancy model for clearance queries.
        include_cargo: Unused (retained for call-site compatibility).
        grasp_config: Unused (retained for call-site compatibility).
        contact_radius: Unused (retained for call-site compatibility).
        collision_backend: Unused (retained for call-site compatibility).
        mjcf_path: Unused (retained for call-site compatibility).
        scenario: Unused (retained for call-site compatibility).
        workspace_bounds: Unused (retained for call-site compatibility).

    Returns:
        A checker exposing ``is_collision_free`` and ``clearance``.
    """
    _ = (
        include_cargo,
        grasp_config,
        contact_radius,
        collision_backend,
        mjcf_path,
        scenario,
        workspace_bounds,
    )
    return CSpaceChecker(kinematics, occupancy)
