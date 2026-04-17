"""Trajectory post-processing chain.

Applies three successive refinement stages to a raw ARCO planner path when
ARCO is installed:

1. ``arco.planning.TrajectoryPruner`` — remove redundant waypoints.
2. ``arco.planning.TrajectoryOptimizer`` — time-optimal refinement with
   collision awareness and an IK hook.
3. ``arco.guidance.BSplineInterpolator`` — C² smooth interpolation.

When ARCO is **not** installed, a pure-Python linear-interpolation fallback
is used instead, producing a ``_JointTrajectory`` compatible with the
``trajectory_msgs/JointTrajectory`` duck-type contract (``joint_names`` list
and ``points`` list of objects with a ``positions`` attribute).

The output is always an object whose ``points`` attribute is a non-empty
list of at least 2 waypoints.

Satisfies requirement FR-PLN-06.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

try:
    from arco.guidance import (
        BSplineInterpolator,
    )
    from arco.planning import (
        TrajectoryOptimizer,
        TrajectoryPruner,
    )
except ImportError:
    TrajectoryOptimizer = None
    TrajectoryPruner = None
    BSplineInterpolator = None

# ---------------------------------------------------------------------------
# Pure-Python trajectory containers (duck-type compatible with ROS msgs)
# ---------------------------------------------------------------------------

_N_INTERP: int = 8  # samples per segment for the linear fallback


@dataclass
class _JointTrajectoryPoint:
    """Minimal JointTrajectoryPoint compatible with ROS duck-type contract."""

    positions: list[float]
    velocities: list[float] = field(default_factory=list)
    time_from_start: float = 0.0


@dataclass
class _JointTrajectory:
    """Minimal JointTrajectory compatible with ROS duck-type contract."""

    joint_names: list[str] = field(default_factory=list)
    points: list[_JointTrajectoryPoint] = field(default_factory=list)


class TrajectoryGenerator:
    """Run the post-processing chain on a raw joint-space path.

    Uses the ARCO chain (``TrajectoryPruner`` → ``TrajectoryOptimizer`` →
    ``BSplineInterpolator``) when ARCO is installed.  Falls back to a
    uniform linear interpolation otherwise, producing a ``_JointTrajectory``
    that satisfies the ``len(traj.points) >= 2`` contract.

    Args:
        kinematics: Kinematics engine used by the optimizer as an IK callable
            and for feasibility checks.  Must expose ``dof: int`` and
            ``joint_names: list[str]``.
    """

    def __init__(self, kinematics: Kinematics) -> None:
        self._kin = kinematics

    def process(self, path: list[npt.NDArray[np.float64]]) -> Any:
        """Apply the post-processing chain to a raw planner path.

        Args:
            path: Raw waypoint sequence; each element shape ``(DOF,)``.
                Must have at least 2 elements.

        Returns:
            A trajectory object whose ``points`` attribute is a non-empty
            list of at least 2 ``JointTrajectoryPoint``-compatible objects.

        Raises:
            ValueError: If ``path`` has fewer than 2 waypoints.
            RuntimeError: If any stage of the ARCO chain raises an
                unrecoverable error.
        """
        if len(path) < 2:
            raise ValueError(
                f"path must have at least 2 waypoints, got {len(path)}"
            )

        if TrajectoryPruner is not None:
            return self._process_arco(path)
        return self._process_linear(path)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _process_linear(
        self, path: list[npt.NDArray[np.float64]]
    ) -> _JointTrajectory:
        """Linear-interpolation fallback used when ARCO is not installed."""
        joint_names = list(self._kin.joint_names)
        points: list[_JointTrajectoryPoint] = []
        n_seg = len(path) - 1
        dt = 1.0 / _N_INTERP
        t = 0.0
        for i in range(n_seg):
            q_start = np.asarray(path[i], dtype=np.float64)
            q_end = np.asarray(path[i + 1], dtype=np.float64)
            for j in range(_N_INTERP):
                alpha = j / _N_INTERP
                q = q_start + alpha * (q_end - q_start)
                points.append(
                    _JointTrajectoryPoint(
                        positions=q.tolist(),
                        time_from_start=t,
                    )
                )
                t += dt
        # Final waypoint
        points.append(
            _JointTrajectoryPoint(
                positions=np.asarray(path[-1], dtype=np.float64).tolist(),
                time_from_start=t,
            )
        )
        return _JointTrajectory(joint_names=joint_names, points=points)

    def _process_arco(
        self, path: list[npt.NDArray[np.float64]]
    ) -> Any:  # pragma: no cover — requires ARCO
        """Full ARCO post-processing chain."""
        pruner = TrajectoryPruner()
        pruned = pruner.prune(path)
        optimizer = TrajectoryOptimizer(ik_hook=self._kin.inverse_kinematics)
        optimized = optimizer.optimize(pruned)
        interpolator = BSplineInterpolator()
        return interpolator.interpolate(optimized)
