"""Joint-space path helpers for pick-place motion segments.

The FSM emits ``PickPlaceCommand.needs_plan`` / ``plan_goal`` on each arm
motion. Runners call :func:`plan_joint_segment` (free space) or the clutter
``PlannerNode`` path when walls are present.
"""

from __future__ import annotations

from typing import Protocol, Sequence

import numpy as np
import numpy.typing as npt


class JointPathPlanner(Protocol):
    """Plan a joint-space path from ``q_start`` to ``q_goal``."""

    def plan(
        self,
        q_start: npt.NDArray[np.float64],
        q_goal: npt.NDArray[np.float64],
    ) -> list[npt.NDArray[np.float64]]:
        """Return a dense waypoint list including start and goal."""


class StraightJointPlanner:
    """Free-space planner: linear interpolation in joint space."""

    def __init__(self, *, n_points: int = 2) -> None:
        if n_points < 2:
            raise ValueError("n_points must be >= 2")
        self._n = int(n_points)

    def plan(
        self,
        q_start: npt.NDArray[np.float64],
        q_goal: npt.NDArray[np.float64],
    ) -> list[npt.NDArray[np.float64]]:
        a = np.asarray(q_start, dtype=np.float64).reshape(-1)
        b = np.asarray(q_goal, dtype=np.float64).reshape(-1)
        if a.shape != b.shape:
            raise ValueError(f"DOF mismatch: {a.shape} vs {b.shape}")
        alphas = np.linspace(0.0, 1.0, self._n)
        return [(1.0 - t) * a + t * b for t in alphas]


def plan_joint_segment(
    q_start: npt.NDArray[np.float64],
    q_goal: npt.NDArray[np.float64],
    *,
    planner: JointPathPlanner | None = None,
    via: Sequence[npt.NDArray[np.float64]] | None = None,
) -> list[npt.NDArray[np.float64]]:
    """Plan ``q_start → [via…] → q_goal`` with the given (or straight) planner."""
    active: JointPathPlanner = (
        planner if planner is not None else StraightJointPlanner()
    )
    waypoints = [np.asarray(q_start, dtype=np.float64).reshape(-1).copy()]
    if via:
        waypoints.extend(
            np.asarray(v, dtype=np.float64).reshape(-1).copy() for v in via
        )
    waypoints.append(np.asarray(q_goal, dtype=np.float64).reshape(-1).copy())
    path: list[npt.NDArray[np.float64]] = []
    for i in range(len(waypoints) - 1):
        segment = active.plan(waypoints[i], waypoints[i + 1])
        if not segment:
            raise RuntimeError("planner returned an empty segment")
        if path:
            # Drop duplicate joint at segment boundary.
            path.extend(segment[1:])
        else:
            path.extend(segment)
    return path
