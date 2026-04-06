"""Trajectory conversion module for ARCO-FRET controller handover (Issue 06).

Provides :class:`TrajectoryConverter`, which converts geometric paths output
by the planner into time-parameterized, kinematically-constrained trajectory
references consumable by the FRET control/IK stack.

**Design invariants:**

- Conversion is deterministic for the same input and configuration.
- All kinematic constraints (velocity, acceleration) are enforced per joint.
- Infeasible conversions are represented via the ``status`` field; this
  module never raises on expected failure conditions.
- The output ``TrajectoryResult`` dict constitutes the planner-to-controller
  handover contract.

**Algorithm:**

Time parameterization uses per-joint trapezoidal velocity profiles
(bang-coast-bang), synchronized across joints by taking the maximum segment
time.  This guarantees that all joints start and stop simultaneously while
each joint individually respects its velocity and acceleration limits.

**Handover Contract** (``TrajectoryResult`` fields):

- ``request_id``     – Propagated or freshly generated UUID string.
- ``status``         – ``"success"`` | ``"infeasible"`` | ``"invalid_input"``.
- ``waypoints``      – Ordered joint configurations (``List[List[float]]``).
- ``timestamps``     – Monotonically non-decreasing times in seconds from
  trajectory start; one entry per waypoint.
- ``velocities``     – Per-joint velocities at each waypoint (rad/s or m/s).
  Zero for a stop-and-go (rest-to-rest) trajectory.
- ``accelerations``  – Per-joint accelerations at each waypoint.
  Zero for a stop-and-go trajectory.
- ``total_time``     – Total trajectory duration in seconds.
- ``waypoint_count`` – Number of waypoints (``== len(waypoints)``).
- ``failure_reason`` – Human-readable explanation; empty string on success.
- ``reference_frame`` – Always ``"world"``.
- ``command_rate_hz`` – Recommended controller update rate in Hz.
"""

from __future__ import annotations

import math
import uuid
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

#: Canonical planning/control reference frame used throughout FRET.
CANONICAL_FRAME = "world"

#: Valid ``status`` values returned in a :class:`TrajectoryResult`.
VALID_STATUSES: frozenset[str] = frozenset(
    {"success", "infeasible", "invalid_input"}
)

#: Default converter configuration.  Missing keys in caller-supplied configs
#: fall back to these values via :func:`_deep_merge`.
DEFAULT_CONFIG: Dict[str, Any] = {
    # Per-joint maximum velocity (rad/s for revolute, m/s for prismatic).
    # A scalar is broadcast to all joints; a list must match the robot DOF.
    "max_velocity": 1.0,
    # Per-joint maximum acceleration (rad/s^2 or m/s^2).
    "max_acceleration": 2.0,
    # Minimum segment duration (seconds).  Prevents numerical issues for
    # near-zero joint displacements.
    "min_segment_time": 1e-3,
    # Recommended controller command update rate (Hz).  Reported verbatim
    # in TrajectoryResult for downstream consumers.
    "command_rate_hz": 100.0,
}


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _deep_merge(
    base: Dict[str, Any], override: Dict[str, Any]
) -> Dict[str, Any]:
    """Recursively merge *override* into a copy of *base*.

    Nested dicts are merged recursively; all other values are replaced by the
    override.  The original *base* dict is never mutated.

    Args:
        base: The base configuration dict.
        override: The override dict whose values take precedence.

    Returns:
        A new dict with merged values.
    """
    result: Dict[str, Any] = dict(base)
    for key, value in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _expand_per_joint(
    value: Any,
    dof: int,
    name: str,
) -> List[float]:
    """Expand a scalar or list config value to a per-joint list of length *dof*.

    Args:
        value: A positive scalar ``float`` or a ``list`` of positive floats.
        dof: Number of joints (target length).
        name: Config key name used in error messages.

    Returns:
        A list of *dof* positive floats.

    Raises:
        ValueError: If *value* is non-positive, has incorrect length, or is
            not a supported type.
    """
    if isinstance(value, (int, float)):
        if value <= 0:
            raise ValueError(f"config '{name}' must be positive, got {value}")
        return [float(value)] * dof
    if isinstance(value, list):
        if len(value) != dof:
            raise ValueError(
                f"config '{name}' has {len(value)} entries, expected {dof}"
            )
        for i, v in enumerate(value):
            if not isinstance(v, (int, float)) or v <= 0:
                raise ValueError(
                    f"config '{name}[{i}]' must be a positive number,"
                    f" got {v!r}"
                )
        return [float(v) for v in value]
    raise ValueError(
        f"config '{name}' must be a float or list of floats,"
        f" got {type(value).__name__}"
    )


def _trapezoid_segment_time(
    distance: float,
    v_max: float,
    a_max: float,
) -> float:
    """Compute minimum feasible time for one joint to travel *distance*.

    Uses a bang-coast-bang (trapezoidal) velocity profile:

    - **Triangular** (no coast phase) when the unconstrained peak velocity
      ``sqrt(distance * a_max)`` does not exceed *v_max*.
    - **Trapezoidal** (saturated at *v_max*) otherwise.

    Args:
        distance: Absolute joint displacement (non-negative).
        v_max: Maximum joint velocity (positive, rad/s or m/s).
        a_max: Maximum joint acceleration (positive, rad/s² or m/s²).

    Returns:
        Minimum feasible duration in seconds (0.0 when *distance* is zero).
    """
    if distance <= 0.0:
        return 0.0
    # Unconstrained peak velocity for a no-coast (triangular) profile.
    v_peak = math.sqrt(distance * a_max)
    if v_peak <= v_max:
        # Triangular profile: t = 2 * v_peak / a_max.
        return 2.0 * v_peak / a_max
    # Trapezoidal profile: ramp up to v_max, coast, ramp down.
    t_ramp = v_max / a_max
    d_ramp = 0.5 * a_max * t_ramp * t_ramp  # distance per ramp phase
    d_coast = distance - 2.0 * d_ramp
    t_coast = d_coast / v_max
    return 2.0 * t_ramp + t_coast


# ---------------------------------------------------------------------------
# Public class
# ---------------------------------------------------------------------------


class TrajectoryConverter:
    """Converts geometric paths into time-parameterized trajectory references.

    The converter applies per-joint trapezoidal velocity profiles, synchronized
    across all joints so that each inter-waypoint segment begins and ends
    simultaneously while each joint individually respects its configured
    velocity and acceleration limits.

    The resulting trajectory is **rest-to-rest** at every waypoint (zero
    velocity at the start and end of each segment), which is the safest
    mode for controller handover and avoids blending discontinuities.

    Args:
        joint_limits: Per-joint ``(lower, upper)`` bounds as a list of
            ``(float, float)`` tuples.  Must be non-empty; each lower bound
            must be strictly less than its upper bound.
        config: Optional configuration dict overriding :data:`DEFAULT_CONFIG`.
            Supported keys: ``max_velocity``, ``max_acceleration``,
            ``min_segment_time``, ``command_rate_hz``.

    Raises:
        ValueError: If *joint_limits* is empty, any bound is inverted, or
            any config value is invalid.

    Example::

        limits = [(-3.14, 3.14), (-1.57, 1.57)]
        converter = TrajectoryConverter(limits)
        result = converter.convert([[0.0, 0.0], [1.0, 0.5]])
        assert result["status"] == "success"
    """

    def __init__(
        self,
        joint_limits: List[Tuple[float, float]],
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not joint_limits:
            raise ValueError("joint_limits must be non-empty")
        for i, (lo, hi) in enumerate(joint_limits):
            if lo >= hi:
                raise ValueError(
                    f"joint_limits[{i}]: lower bound {lo} must be strictly"
                    f" less than upper bound {hi}"
                )

        self._joint_limits: List[Tuple[float, float]] = list(joint_limits)
        self._dof: int = len(joint_limits)

        merged: Dict[str, Any] = _deep_merge(DEFAULT_CONFIG, config or {})
        self._config: Dict[str, Any] = merged

        self._v_max: List[float] = _expand_per_joint(
            merged["max_velocity"], self._dof, "max_velocity"
        )
        self._a_max: List[float] = _expand_per_joint(
            merged["max_acceleration"], self._dof, "max_acceleration"
        )
        self._min_segment_time: float = float(merged["min_segment_time"])
        self._command_rate_hz: float = float(merged["command_rate_hz"])

        if self._min_segment_time <= 0:
            raise ValueError(
                "config 'min_segment_time' must be positive,"
                f" got {self._min_segment_time}"
            )
        if self._command_rate_hz <= 0:
            raise ValueError(
                "config 'command_rate_hz' must be positive,"
                f" got {self._command_rate_hz}"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def convert(
        self,
        path: List[List[float]],
        request_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Convert a geometric path into a time-parameterized trajectory.

        The conversion enforces per-joint velocity and acceleration limits via
        trapezoidal time parameterization.  All joints move synchronously:
        every segment begins and ends at the same instant.

        Args:
            path: Ordered list of joint configurations (waypoints).  Each
                element is a list of exactly *dof* floats representing the
                target joint values for that waypoint.
            request_id: Optional UUID string propagated from a
                ``PlanningResult`` (handover contract).  A fresh UUID is
                generated when not provided.

        Returns:
            ``TrajectoryResult`` dict with the following keys:

            - ``request_id`` (``str``): Echoed or freshly generated UUID.
            - ``status`` (``str``): ``"success"``, ``"infeasible"``, or
              ``"invalid_input"``.
            - ``waypoints`` (``List[List[float]]``): Copy of *path*.
            - ``timestamps`` (``List[float]``): Non-decreasing times in
              seconds from trajectory start; one value per waypoint.
            - ``velocities`` (``List[List[float]]``): Per-joint velocities
              at each waypoint.  Zero for stop-and-go trajectories.
            - ``accelerations`` (``List[List[float]]``): Per-joint
              accelerations at each waypoint.  Zero for stop-and-go.
            - ``total_time`` (``float``): Total trajectory duration (s).
            - ``waypoint_count`` (``int``): ``len(waypoints)``.
            - ``failure_reason`` (``str``): Empty on success; human-readable
              explanation on failure.
            - ``reference_frame`` (``str``): Always ``"world"``.
            - ``command_rate_hz`` (``float``): Recommended update rate.
        """
        rid: str = request_id if request_id is not None else str(uuid.uuid4())
        zero_dof: List[float] = [0.0] * self._dof

        base: Dict[str, Any] = {
            "request_id": rid,
            "status": "invalid_input",
            "waypoints": [],
            "timestamps": [],
            "velocities": [],
            "accelerations": [],
            "total_time": 0.0,
            "waypoint_count": 0,
            "failure_reason": "",
            "reference_frame": CANONICAL_FRAME,
            "command_rate_hz": self._command_rate_hz,
        }

        # --- Input validation ----------------------------------------
        if not path:
            base["failure_reason"] = "path is empty"
            return base

        for idx, wp in enumerate(path):
            if len(wp) != self._dof:
                base["failure_reason"] = (
                    f"waypoint {idx} has {len(wp)} values,"
                    f" expected {self._dof}"
                )
                return base

        # Joint-limit check: infeasible (not a schema error).
        for idx, wp in enumerate(path):
            for j, (val, (lo, hi)) in enumerate(zip(wp, self._joint_limits)):
                if not (lo <= val <= hi):
                    base["status"] = "infeasible"
                    base["failure_reason"] = (
                        f"waypoint {idx} joint {j} value {val:.6f} is"
                        f" outside limits [{lo:.6f}, {hi:.6f}]"
                    )
                    return base

        # --- Time parameterization -----------------------------------
        n: int = len(path)

        if n == 1:
            # Trivial trajectory: stay at the single waypoint.
            base.update(
                {
                    "status": "success",
                    "waypoints": [list(path[0])],
                    "timestamps": [0.0],
                    "velocities": [list(zero_dof)],
                    "accelerations": [list(zero_dof)],
                    "total_time": 0.0,
                    "waypoint_count": 1,
                }
            )
            return base

        timestamps: List[float] = [0.0]
        for seg_idx in range(n - 1):
            q_a = path[seg_idx]
            q_b = path[seg_idx + 1]
            t_seg = 0.0
            for j in range(self._dof):
                dist_j = abs(q_b[j] - q_a[j])
                t_j = _trapezoid_segment_time(
                    dist_j, self._v_max[j], self._a_max[j]
                )
                if t_j > t_seg:
                    t_seg = t_j
            # Enforce minimum segment time to prevent numerical issues.
            if t_seg < self._min_segment_time:
                t_seg = self._min_segment_time
            timestamps.append(timestamps[-1] + t_seg)

        total_time: float = timestamps[-1]

        # Stop-and-go trajectory: velocity and acceleration are zero at
        # every waypoint (rest-to-rest per segment).
        velocities: List[List[float]] = [list(zero_dof) for _ in range(n)]
        accelerations: List[List[float]] = [list(zero_dof) for _ in range(n)]

        base.update(
            {
                "status": "success",
                "waypoints": [list(wp) for wp in path],
                "timestamps": timestamps,
                "velocities": velocities,
                "accelerations": accelerations,
                "total_time": total_time,
                "waypoint_count": n,
            }
        )
        return base
