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

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

try:
    from arco.guidance import BSplineInterpolator
    from arco.planning import TrajectoryOptimizer, TrajectoryPruner
except ImportError:
    TrajectoryOptimizer = None
    TrajectoryPruner = None
    BSplineInterpolator = None

# ---------------------------------------------------------------------------
# Pure-Python trajectory containers (duck-type compatible with ROS msgs)
# ---------------------------------------------------------------------------

# Maximum allowed Cartesian EE step between consecutive waypoints.
#
# Must be chosen so that the steady-state tracking lag (step / Jacobian_gain)
# stays below the controller's fault_threshold (0.020 m).
#
# Derivation for SCARA at q=0 (y-dominant direction, worst-case gain):
#   ΔEE_per_step ≈ 0.330 × error  →  equilibrium error e* = step / 0.330
#   Require  e* < fault_threshold  →  step < 0.020 × 0.330 ≈ 0.0066 m
#
# 4 mm (0.004 m) gives e* ≈ 12 mm, a 2× safety margin below 20 mm.
_MAX_INTERP_STEP_M: float = 0.004  # 4 mm


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
        """Linear-interpolation fallback used when ARCO is not installed.

        The number of interpolation steps per segment is computed from the
        Cartesian end-effector distance between the segment's endpoints so
        that consecutive waypoints are always separated by at most
        ``_MAX_INTERP_STEP_M`` metres.  This guarantees the step is below
        the controller's ``fault_threshold`` (0.020 m) and prevents the
        Jacobian controller from entering HALTED state simply because it
        advanced to a reference that is too far from the robot's EE.
        """
        joint_names = list(self._kin.joint_names)
        points: list[_JointTrajectoryPoint] = []
        n_seg = len(path) - 1
        t = 0.0
        for i in range(n_seg):
            q_start = np.asarray(path[i], dtype=np.float64)
            q_end = np.asarray(path[i + 1], dtype=np.float64)

            # Compute the Cartesian EE distance for this segment.
            x_start = self._kin.forward_kinematics(q_start)[:3, 3]
            x_end = self._kin.forward_kinematics(q_end)[:3, 3]
            cart_dist = float(np.linalg.norm(x_end - x_start))

            # Number of sub-steps: at least 1, enough to keep each step ≤ max.
            # A 10 % curvature margin compensates for the nonlinear EE arc that
            # results from linear-in-joint-space interpolation.
            n_steps = max(math.ceil(cart_dist / _MAX_INTERP_STEP_M * 1.1), 1)
            dt = 1.0 / n_steps

            for j in range(n_steps):
                alpha = j / n_steps
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
    ) -> Any:  # pragma: no cover - requires ARCO
        """Full ARCO post-processing chain."""
        pruner = TrajectoryPruner()
        pruned = pruner.prune(path)
        optimizer = TrajectoryOptimizer(ik_hook=self._kin.inverse_kinematics)
        optimized = optimizer.optimize(pruned)
        interpolator = BSplineInterpolator()
        return interpolator.interpolate(optimized)
