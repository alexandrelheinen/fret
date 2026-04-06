"""Abstract robot kinematic model base class for the ARCO-FRET pipeline.

All concrete robot models (SCARA, Dubins vehicle, PPP, …) must inherit
from :class:`RobotModel` and implement the three abstract members.

Design invariants:
    - Pure Python; no ROS 2 dependency.
    - :meth:`forward_kinematics` returns a list of **control points** in
      the canonical ``world`` frame.  Each control point is a ``[x, y, z]``
      triple (metres).  Callers use these points to query occupancy.
    - :meth:`make_state_validator` wires the kinematic model to an
      :class:`~fret.perception.OccupancyAdapter` instance and returns a
      plain callable suitable for :class:`~fret.planning.PlannerAdapter`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from fret.perception.occupancy_adapter import OccupancyAdapter


class RobotModel(ABC):
    """Abstract base class for robot kinematic models.

    Concrete subclasses implement the three abstract members to expose
    the information that the ARCO planning pipeline needs:

    1. The number of planning degrees of freedom.
    2. Per-joint position limits as (lower, upper) bounds.
    3. A forward-kinematics function that maps a joint configuration to a
       list of Cartesian control points in the world frame.

    Example (SCARA subclass sketch)::

        class ScaraModel(RobotModel):
            @property
            def n_joints(self) -> int:
                return 4

            @property
            def joint_limits(self) -> list[tuple[float, float]]:
                return [(-2.304, 2.304), (-2.618, 2.618), (0.0, 0.2),
                        (-3.14159, 3.14159)]

            def forward_kinematics(
                self, q: list[float]
            ) -> list[list[float]]:
                # ... returns sampled control points for each link ...
    """

    @property
    @abstractmethod
    def n_joints(self) -> int:
        """Number of planning degrees of freedom."""

    @property
    @abstractmethod
    def joint_limits(self) -> list[tuple[float, float]]:
        """Per-joint position limits as a list of (lower, upper) pairs.

        Returns:
            List of (lower, upper) bound pairs, one per joint, in the
            same order as the joint configuration vector ``q``.
        """

    @abstractmethod
    def forward_kinematics(self, q: list[float]) -> list[list[float]]:
        """Compute Cartesian control points for configuration *q*.

        Args:
            q: Joint configuration vector (length == :attr:`n_joints`).
               Units follow the URDF convention (radians for revolute,
               metres for prismatic joints).

        Returns:
            List of ``[x, y, z]`` triples (metres) in the canonical
            ``world`` frame.  The list must contain at least the
            end-effector position, but should also include intermediate
            link positions to enable accurate collision checking along the
            full kinematic chain.
        """

    def make_state_validator(
        self,
        occupancy_adapter: "OccupancyAdapter",
    ) -> Callable[[list[float]], bool]:
        """Return a state-validator function bound to *occupancy_adapter*.

        The returned callable maps a joint configuration to ``True`` when
        all FK control points lie in free space according to
        *occupancy_adapter*, and to ``False`` otherwise.  Suitable for
        passing directly to
        :class:`~fret.planning.PlannerAdapter` as ``state_validator``.

        Args:
            occupancy_adapter: An :class:`~fret.perception.OccupancyAdapter`
                instance used to query obstacle occupancy.

        Returns:
            A callable ``(q: list[float]) -> bool``.
        """

        def validator(q: list[float]) -> bool:
            points = self.forward_kinematics(q)
            return all(occupancy_adapter.is_free(p) for p in points)

        return validator
