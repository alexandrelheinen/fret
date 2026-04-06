"""Metric computation utilities for ARCO-FRET validation benchmarks.

Provides deterministic, ROS-free functions for computing planning and
execution quality metrics defined in
``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md``.

**Metrics implemented:**

- :func:`path_length` — joint-space arc length of a planned path.
- :func:`path_smoothness` — total direction-change angle along a path.
- :func:`min_obstacle_clearance` — minimum clearance from any waypoint to
  the nearest obstacle.
- :func:`tracking_rmse` — root mean squared joint-space tracking error.

All functions accept plain Python lists; no NumPy or ROS dependency is
required.  The functions are deterministic: equal inputs always produce
equal outputs.

See also:
    ``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md`` —
    full specification, metric formulas, and sampling-window definitions.
"""

from __future__ import annotations

import math
from typing import Callable, List

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Waypoint = List[float]
Path = List[Waypoint]
ClearanceFn = Callable[[Waypoint], float]


# ---------------------------------------------------------------------------
# Metric functions
# ---------------------------------------------------------------------------


def path_length(path: Path) -> float:
    """Compute the joint-space arc length of a path.

    The arc length is the sum of Euclidean distances between consecutive
    waypoints in joint space.  For revolute joints the distance is in
    radians; for prismatic joints it is in meters.

    Args:
        path: Ordered sequence of joint configurations.  Each waypoint is a
            list of ``n`` floats (joint positions).  An empty path or a
            single-waypoint path returns ``0.0``.

    Returns:
        Total path length in the joint-space metric (mixed rad/m units for
        heterogeneous joint types).

    Raises:
        ValueError: If waypoints within the same path have inconsistent
            lengths (different DOF).
    """
    if len(path) < 2:
        return 0.0

    dof = len(path[0])
    total = 0.0
    for i in range(1, len(path)):
        wp = path[i]
        if len(wp) != dof:
            raise ValueError(
                f"Waypoint {i} has {len(wp)} joints; "
                f"expected {dof} (from waypoint 0)"
            )
        prev = path[i - 1]
        dist = math.sqrt(sum((wp[j] - prev[j]) ** 2 for j in range(dof)))
        total += dist
    return total


def path_smoothness(path: Path) -> float:
    """Compute path smoothness as the total direction-change angle.

    For each consecutive triple of waypoints (A, B, C) the function
    computes the angle between direction vectors :math:`\\overrightarrow{AB}`
    and :math:`\\overrightarrow{BC}` and accumulates the sum.  A perfectly
    straight path returns ``0.0``.  Higher values indicate sharper turns.

    The angle at each triple is clamped to ``[0, π]`` by the arccos
    formula, so collinear waypoints contribute ``0.0`` and fully reversed
    segments contribute ``π``.

    Args:
        path: Ordered sequence of joint configurations.  Paths with fewer
            than three waypoints return ``0.0``.

    Returns:
        Total direction-change angle in radians.  ``0.0`` for a straight
        or near-straight path.

    Raises:
        ValueError: If waypoints within the same path have inconsistent
            lengths.
    """
    if len(path) < 3:
        return 0.0

    dof = len(path[0])
    total = 0.0

    for i in range(1, len(path) - 1):
        a = path[i - 1]
        b = path[i]
        c = path[i + 1]

        if len(b) != dof or len(c) != dof:
            raise ValueError(
                "All waypoints must have the same DOF as waypoint 0"
            )

        ab = [b[j] - a[j] for j in range(dof)]
        bc = [c[j] - b[j] for j in range(dof)]

        norm_ab = math.sqrt(sum(x * x for x in ab))
        norm_bc = math.sqrt(sum(x * x for x in bc))

        if norm_ab < 1e-12 or norm_bc < 1e-12:
            # Zero-length segment: no direction defined, contribute 0.
            continue

        dot = sum(ab[j] * bc[j] for j in range(dof))
        # Clamp to [-1, 1] to guard against floating-point rounding.
        cos_angle = max(-1.0, min(1.0, dot / (norm_ab * norm_bc)))
        total += math.acos(cos_angle)

    return total


def min_obstacle_clearance(
    path: Path,
    clearance_fn: ClearanceFn,
) -> float:
    """Compute the minimum obstacle clearance along a path.

    Queries ``clearance_fn`` at every waypoint and returns the smallest
    value.  A clearance of ``0.0`` means the waypoint is in or on the
    surface of an inflated obstacle.  A clearance of ``math.inf`` is
    returned for empty paths (no waypoints to evaluate).

    Args:
        path: Ordered sequence of joint configurations (or proxy
            Cartesian points, depending on the ``clearance_fn`` contract).
        clearance_fn: Callable ``(waypoint: list[float]) -> float`` that
            returns the obstacle clearance at the given point in meters.
            For occupancy-adapter–based callers, pass
            ``OccupancyAdapter.clearance``.

    Returns:
        Minimum clearance in meters across all waypoints.  Returns
        ``math.inf`` when ``path`` is empty.
    """
    if not path:
        return math.inf

    return min(clearance_fn(wp) for wp in path)


def tracking_rmse(
    actual: List[Waypoint],
    desired: List[Waypoint],
) -> float:
    """Compute root mean squared joint-space tracking error.

    The RMSE is defined as:

    .. math::

        \\text{RMSE} = \\sqrt{\\frac{1}{N \\cdot D}
                       \\sum_{k=1}^{N} \\sum_{j=1}^{D} (a_{kj} - d_{kj})^2}

    where *N* is the number of time steps, *D* is the DOF, :math:`a_{kj}`
    is the actual joint-*j* position at step *k*, and :math:`d_{kj}` is
    the desired joint-*j* position at step *k*.

    Args:
        actual: Sequence of actual joint-position samples, one list per
            time step.
        desired: Sequence of desired joint-position samples at the same
            time steps.  Must have the same length as ``actual``.

    Returns:
        RMSE in mixed rad/m units (same units as the joint positions).

    Raises:
        ValueError: If ``actual`` and ``desired`` have different lengths,
            if either sequence is empty, or if waypoints have inconsistent
            DOF.
    """
    if len(actual) != len(desired):
        raise ValueError(
            f"actual ({len(actual)} steps) and desired ({len(desired)} steps)"
            " must have the same length"
        )
    if not actual:
        raise ValueError("actual and desired must not be empty")

    dof = len(actual[0])
    if dof == 0:
        raise ValueError("waypoints must have at least one joint (DOF >= 1)")

    total_sq = 0.0
    for k, (a, d) in enumerate(zip(actual, desired)):
        if len(a) != dof or len(d) != dof:
            raise ValueError(
                f"Step {k}: all waypoints must have the same DOF ({dof})"
            )
        total_sq += sum((a[j] - d[j]) ** 2 for j in range(dof))

    return math.sqrt(total_sq / (len(actual) * dof))
