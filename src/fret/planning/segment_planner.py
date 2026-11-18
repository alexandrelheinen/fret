"""Joint-space RRT whose goal is a task-space line segment.

The goal is not one configuration. A node is a goal when its end-effector
position lies within ``goal_tolerance_m`` of the segment and the
caller-supplied predicate says the configuration is free. Goal bias draws
inverse-kinematics solutions of points spread along the segment, lowest
heights first, so the path stops at the first height the arm can reach.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Protocol

import numpy as np
import numpy.typing as npt

from fret.planning.goal_segment import GoalSegment


class SegmentKinematics(Protocol):
    """FK / IK surface used by :func:`plan_to_segment`."""

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


class SegmentPlanningError(RuntimeError):
    """Raised when no collision-free path reaches the goal segment."""


def _ee_position(
    kinematics: SegmentKinematics,
    configuration: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    pose = kinematics.forward_kinematics(configuration)
    return np.asarray(pose[:3, 3], dtype=np.float64)


def _on_segment(
    kinematics: SegmentKinematics,
    configuration: npt.NDArray[np.float64],
    segment: GoalSegment,
    tolerance_m: float,
) -> bool:
    return (
        float(segment.distance(_ee_position(kinematics, configuration)))
        <= tolerance_m
    )


def _steer(
    source: npt.NDArray[np.float64],
    target: npt.NDArray[np.float64],
    step: float,
    limits: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    delta = target - source
    distance = float(np.linalg.norm(delta))
    if distance <= step or distance <= 1e-12:
        stepped = target.copy()
    else:
        stepped = source + delta * (step / distance)
    return np.clip(stepped, limits[:, 0], limits[:, 1])


def _edge_free(
    start: npt.NDArray[np.float64],
    end: npt.NDArray[np.float64],
    is_free: Callable[[npt.NDArray[np.float64]], bool],
    *,
    samples: int,
) -> bool:
    count = max(2, int(samples))
    for fraction in np.linspace(0.0, 1.0, count):
        configuration = (1.0 - fraction) * start + fraction * end
        if not is_free(configuration):
            return False
    return True


def _ik_goals(
    start: npt.NDArray[np.float64],
    segment: GoalSegment,
    kinematics: SegmentKinematics,
    is_free: Callable[[npt.NDArray[np.float64]], bool],
    *,
    tolerance_m: float,
    fractions: int,
) -> list[npt.NDArray[np.float64]]:
    """IK solutions on the segment, ordered from ``start`` of the segment."""
    limits = kinematics.joint_limits
    seeds: list[npt.NDArray[np.float64] | None] = [
        np.asarray(start, dtype=np.float64).copy(),
        limits.mean(axis=1),
    ]
    goals: list[npt.NDArray[np.float64]] = []
    for fraction in np.linspace(0.0, 1.0, max(2, int(fractions))):
        target = segment.point_at(float(fraction))
        for seed in seeds:
            try:
                configuration = np.asarray(
                    kinematics.inverse_kinematics(target, seed=seed),
                    dtype=np.float64,
                ).reshape(-1)
            except ValueError:
                # Kinematics raises when the sample is outside the workspace.
                continue
            if configuration.shape != (limits.shape[0],):
                continue
            if not _on_segment(
                kinematics, configuration, segment, tolerance_m
            ):
                continue
            if not is_free(configuration):
                continue
            goals.append(configuration.copy())
            break
    return goals


def _reconstruct(
    parents: list[int],
    tree: list[npt.NDArray[np.float64]],
    index: int,
) -> list[npt.NDArray[np.float64]]:
    chain: list[npt.NDArray[np.float64]] = []
    cursor = index
    while cursor >= 0:
        chain.append(tree[cursor].copy())
        cursor = parents[cursor]
    chain.reverse()
    return chain


def plan_to_segment(
    start: npt.NDArray[np.float64],
    segment: GoalSegment,
    *,
    kinematics: SegmentKinematics,
    is_free: Callable[[npt.NDArray[np.float64]], bool],
    seed: int = 0,
    timeout_s: float = 5.0,
    goal_tolerance_m: float = 0.005,
    step_size: float = 0.35,
    max_samples: int = 2000,
) -> list[npt.NDArray[np.float64]]:
    """Plan a joint path from ``start`` to any point of ``segment``.

    Args:
        start: Joint configuration, shape ``(DOF,)``.
        segment: Goal region. Every point of the segment is a valid
            end-effector position.
        kinematics: Forward and inverse kinematics for the arm.
        is_free: ``is_free(q) -> bool``. Collision predicate in joint space.
        seed: RNG seed for samples that are not inverse-kinematics goals.
        timeout_s: Wall-clock budget for the search.
        goal_tolerance_m: How close the end effector must lie to the
            segment to count as a goal [m].
        step_size: Maximum joint-space step [rad].
        max_samples: Cap on RRT samples after the direct connections.

    Returns:
        At least two joint configurations. The last one places the end
        effector on ``segment``. Direct connections try the low end of the
        segment first.

    Raises:
        SegmentPlanningError: When ``start`` is in collision or no path
            reaches the segment inside the budget.
    """
    limits = np.asarray(kinematics.joint_limits, dtype=np.float64)
    start_q = (
        np.asarray(start, dtype=np.float64).reshape(limits.shape[0]).copy()
    )
    if not is_free(start_q):
        raise SegmentPlanningError("start configuration is not collision-free")

    goals = _ik_goals(
        start_q,
        segment,
        kinematics,
        is_free,
        tolerance_m=goal_tolerance_m,
        fractions=9,
    )
    for goal in goals:
        if _edge_free(start_q, goal, is_free, samples=8):
            return [start_q.copy(), goal.copy()]

    rng = np.random.default_rng(int(seed))
    tree: list[npt.NDArray[np.float64]] = [start_q.copy()]
    parents = [-1]
    deadline = time.monotonic() + float(timeout_s)
    for _ in range(int(max_samples)):
        if time.monotonic() > deadline:
            break
        if goals and float(rng.random()) < 0.45:
            sample = goals[int(rng.integers(0, len(goals)))]
        else:
            sample = rng.uniform(limits[:, 0], limits[:, 1])
        stacked = np.stack(tree, axis=0)
        nearest = int(np.argmin(np.linalg.norm(stacked - sample, axis=1)))
        proposed = _steer(tree[nearest], sample, float(step_size), limits)
        if not _edge_free(tree[nearest], proposed, is_free, samples=6):
            continue
        tree.append(proposed)
        parents.append(nearest)
        if _on_segment(kinematics, proposed, segment, goal_tolerance_m):
            return _reconstruct(parents, tree, len(tree) - 1)
        if goals:
            goal = goals[int(rng.integers(0, len(goals)))]
            if _edge_free(proposed, goal, is_free, samples=6):
                tree.append(goal.copy())
                parents.append(len(tree) - 2)
                return _reconstruct(parents, tree, len(tree) - 1)

    raise SegmentPlanningError(
        "no collision-free path reaches the goal segment "
        f"(ik_goals={len(goals)}, tree={len(tree)})"
    )
