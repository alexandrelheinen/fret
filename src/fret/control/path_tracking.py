"""Arc-length carrot tracking helpers (ARCO PPP race pattern).

Provides reusable utilities for smooth joint-space execution along dense
reference paths without stop-and-go waypoint holds.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
import numpy.typing as npt

_EPSILON = 1e-9


def cumulative_arc_lengths(path: list[npt.NDArray[np.float64]]) -> list[float]:
    """Return cumulative arc lengths along *path* starting at 0.0."""
    if not path:
        return [0.0]
    arcs = [0.0]
    for i in range(len(path) - 1):
        arcs.append(arcs[-1] + float(np.linalg.norm(path[i + 1] - path[i])))
    return arcs


def sample_path_at_distance(
    path: list[npt.NDArray[np.float64]],
    arcs: list[float],
    dist: float,
) -> tuple[npt.NDArray[np.float64], bool]:
    """Interpolate configuration at arc-length *dist* along *path*."""
    if not path:
        raise ValueError("path must not be empty")
    if dist >= arcs[-1]:
        return path[-1].copy(), True
    for i in range(len(arcs) - 1):
        if arcs[i + 1] >= dist:
            seg = arcs[i + 1] - arcs[i]
            t = (dist - arcs[i]) / max(seg, _EPSILON)
            return path[i] + t * (path[i + 1] - path[i]), False
    return path[-1].copy(), True


def densify_polyline(
    path: list[npt.NDArray[np.float64]],
    max_step: float,
) -> list[npt.NDArray[np.float64]]:
    """Insert intermediate points so no segment exceeds *max_step*."""
    if len(path) < 2:
        return [p.copy() for p in path]
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")

    dense: list[npt.NDArray[np.float64]] = [path[0].copy()]
    for i in range(len(path) - 1):
        q0 = path[i]
        q1 = path[i + 1]
        seg_len = float(np.linalg.norm(q1 - q0))
        if seg_len <= max_step:
            if i < len(path) - 2 or dense[-1] is not q1:
                dense.append(q1.copy())
            continue
        steps = max(1, int(np.ceil(seg_len / max_step)))
        for step in range(1, steps + 1):
            alpha = step / steps
            dense.append((1.0 - alpha) * q0 + alpha * q1)
    return dense


def _nearest_polyline_distance(
    point: npt.NDArray[np.float64],
    path: list[npt.NDArray[np.float64]],
) -> float:
    """Return the shortest distance from *point* to the polyline *path*."""
    if not path:
        return 0.0
    q = np.asarray(point, dtype=np.float64)
    best = float("inf")
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom <= _EPSILON:
            best = min(best, float(np.linalg.norm(q - a)))
            continue
        t = float(np.clip(np.dot(q - a, ab) / denom, 0.0, 1.0))
        proj = a + t * ab
        best = min(best, float(np.linalg.norm(q - proj)))
    return best


def _segment_cross_track_error(
    point: npt.NDArray[np.float64],
    start: npt.NDArray[np.float64],
    end: npt.NDArray[np.float64],
) -> float:
    """Distance from *point* to the segment ``start→end``."""
    q = np.asarray(point, dtype=np.float64)
    a = start
    b = end
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= _EPSILON:
        return float(np.linalg.norm(q - a))
    t = float(np.clip(np.dot(q - a, ab) / denom, 0.0, 1.0))
    proj = a + t * ab
    return float(np.linalg.norm(q - proj))


def _subsample_path(
    path: list[npt.NDArray[np.float64]],
    max_points: int,
) -> list[npt.NDArray[np.float64]]:
    """Downsample an already-dense reference path for carrot tracking."""
    if len(path) <= max_points:
        return [p.copy() for p in path]
    indices = np.linspace(0, len(path) - 1, max_points, dtype=int)
    sampled = [path[int(i)].copy() for i in indices]
    sampled[-1] = path[-1].copy()
    sampled[0] = path[0].copy()
    return sampled


def simulate_joint_carrot_tracking(
    path: list[npt.NDArray[np.float64]],
    *,
    start: npt.NDArray[np.float64],
    race_speed: float,
    max_joint_velocity: npt.NDArray[np.float64],
    max_joint_acc: npt.NDArray[np.float64],
    proportional_gain: float,
    max_carrot_lag: float,
    dt: float,
    goal: npt.NDArray[np.float64] | None = None,
    goal_tolerance: float = 0.02,
    occupancy: object | None = None,
    repulsion_gain: float = 0.0,
    on_step: Callable[[npt.NDArray[np.float64]], None] | None = None,
) -> tuple[list[npt.NDArray[np.float64]], float]:
    """Track *path* with an advancing arc-length carrot (ARCO PPP race).

    Returns:
        Joint history and maximum cross-track error to the reference polyline [m].
    """
    from arco.control import JointSpaceTracker

    nav = [np.asarray(p, dtype=np.float64) for p in path]
    if len(nav) < 2:
        return [start.copy()], 0.0

    arcs = cumulative_arc_lengths(nav)
    tracker = JointSpaceTracker(
        max_vel=max_joint_velocity,
        max_acc=max_joint_acc,
        proportional_gain=proportional_gain,
        occupancy=occupancy,
        repulsion_gain=repulsion_gain,
    )
    tracker.reset(start)
    carrot_dist = 0.0
    carrot, _ = sample_path_at_distance(nav, arcs, carrot_dist)
    history = [tracker.q.copy()]
    max_err_m = 0.0

    max_steps = int((arcs[-1] / max(race_speed * dt, 1e-6)) * 8.0) + 5_000
    for _ in range(max_steps):
        lag = float(np.linalg.norm(tracker.q - carrot))
        if lag < max_carrot_lag:
            carrot_dist = min(carrot_dist + race_speed * dt, arcs[-1])
        carrot, at_path_end = sample_path_at_distance(nav, arcs, carrot_dist)
        tracker.step(carrot, dt)
        path_err_m = float(np.linalg.norm(tracker.q - carrot))
        max_err_m = max(max_err_m, path_err_m)
        history.append(tracker.q.copy())
        if on_step is not None:
            on_step(tracker.q)
        if at_path_end and float(np.linalg.norm(tracker.q - nav[-1])) < goal_tolerance:
            break

    return history, max_err_m
