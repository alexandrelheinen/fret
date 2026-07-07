"""Trajectory post-processing chain.

When ARCO is installed, applies ``arco.planning.TrajectoryPruner`` to remove
redundant waypoints from the raw planner path, then applies the same linear
interpolation used in the fallback path (see below).

Note: ``arco.planning.TrajectoryOptimizer`` and
``arco.guidance.BSplineInterpolator`` are stubs in ARCO 0.3.x and do not yet
produce usable waypoint sequences, so they are intentionally omitted.

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
    from arco.planning import TrajectoryPruner
except ImportError:
    TrajectoryPruner = None

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

    Uses ``TrajectoryPruner`` to remove redundant waypoints when ARCO is
    installed, then applies linear interpolation to produce dense waypoints
    that satisfy the controller's maximum Cartesian step constraint.
    Falls back to linear interpolation only when ARCO is absent.

    Args:
        kinematics: Kinematics engine.  Must expose ``dof: int`` and
            ``joint_names: list[str]``.
    """

    def __init__(self, kinematics: Kinematics) -> None:
        self._kin = kinematics
        self._prune_occupancy: Any | None = None
        self._prune_step_size: npt.NDArray[np.float64] | None = None

    def set_collision_context(
        self,
        occupancy: Any,
        step_size: npt.NDArray[np.float64],
    ) -> None:
        """Provide occupancy and step size for ARCO ``TrajectoryPruner``.

        Args:
            occupancy: Object exposing ``is_occupied(q)`` for joint configs.
            step_size: Per-joint prune step sizes, shape ``(DOF,)``.
        """
        step = np.asarray(step_size, dtype=np.float64)
        if step.shape != (self._kin.dof,):
            raise ValueError(
                f"step_size must have shape ({self._kin.dof},), got {step.shape}"
            )
        self._prune_occupancy = occupancy
        self._prune_step_size = step

    def clear_collision_context(self) -> None:
        """Remove collision context so pruning falls back to linear only."""
        self._prune_occupancy = None
        self._prune_step_size = None

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

        if TrajectoryPruner is not None and self._prune_occupancy is not None:
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
        """Full ARCO post-processing chain.

        Prunes redundant waypoints with ``TrajectoryPruner``, then applies
        the same linear interpolation used in the fallback path so that the
        output satisfies the controller's maximum step constraint.
        ``TrajectoryOptimizer`` and ``BSplineInterpolator`` are stubs in ARCO
        0.3.x and do not yet return usable waypoint sequences, so they are
        intentionally omitted here.
        """
        pruner = TrajectoryPruner(
            self._prune_occupancy,
            self._prune_step_size,
        )
        pruned = pruner.prune(path)
        return self._process_linear(pruned)
