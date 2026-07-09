"""Time-parameterized trajectory conversion for joint-space paths.

Converts a raw joint-space path (list of ``np.ndarray`` waypoints) into a
time-parameterized trajectory using per-joint trapezoidal velocity profiles.

When the motion distance is too short for a full trapezoid, a triangular
(peak-limited) profile is used instead.  The resulting trajectory is
resampled at ``control_hz`` for smooth controller output.

Satisfies requirement FR-PLN-08.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import require_keys

# ---------------------------------------------------------------------------
# TrajectoryConverter
# ---------------------------------------------------------------------------


_REQUIRED_TRAJECTORY_KEYS: tuple[str, ...] = (
    "control_hz",
    "v_max",
    "a_max",
    "dt_min",
    "joint_names",
)


@dataclass
class TrajectoryResult:
    """Time-parameterized trajectory produced by ``TrajectoryConverter``.

    Attributes:
        joint_names: URDF joint names in the same order as the position /
            velocity arrays.
        positions: Resampled joint positions; each element shape ``(DOF,)``.
        velocities: Resampled joint velocities; each element shape ``(DOF,)``.
        timestamps: Absolute timestamps in seconds (from ``start_time``);
            strictly increasing.
        duration: Total trajectory duration in seconds.
    """

    joint_names: list[str]
    positions: list[npt.NDArray[np.float64]] = field(default_factory=list)
    velocities: list[npt.NDArray[np.float64]] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    duration: float = 0.0


# ---------------------------------------------------------------------------
# TrajectoryConverter
# ---------------------------------------------------------------------------


class TrajectoryConverter:
    """Convert a raw joint-space path to a time-parameterized trajectory.

    Uses per-joint trapezoidal velocity profiles to compute segment times and
    then resamples the resulting piecewise profile at ``control_hz`` for
    smooth controller output.

    Args:
        config: Configuration dict.  Accepted keys:

            - ``control_hz`` (float): Output sample rate in Hz.
            - ``v_max`` (list[float]): Per-joint max velocity.
            - ``a_max`` (list[float]): Per-joint max acceleration.
            - ``dt_min`` (float): Minimum segment duration in seconds.
            - ``joint_names`` (list[str]): URDF joint names.

            Values may also be nested under a ``"trajectory"`` sub-key.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        cfg = self._parse_config(config)
        self._control_hz: float = float(cfg["control_hz"])
        self._v_max: npt.NDArray[np.float64] = np.asarray(
            cfg["v_max"], dtype=np.float64
        )
        self._a_max: npt.NDArray[np.float64] = np.asarray(
            cfg["a_max"], dtype=np.float64
        )
        self._dt_min: float = float(cfg["dt_min"])
        self._dof: int = len(self._v_max)
        self._joint_names: list[str] = list(cfg["joint_names"][: self._dof])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def convert(
        self,
        path: list[npt.NDArray[np.float64]],
        start_time: float = 0.0,
    ) -> TrajectoryResult:
        """Convert a joint-space path to a time-parameterized trajectory.

        Args:
            path: Raw waypoint sequence; each element shape ``(DOF,)``.
                Must contain at least 2 waypoints.
            start_time: Absolute start time in seconds.  The first timestamp
                in the result will equal ``start_time``.

        Returns:
            A ``TrajectoryResult`` with at least 2 positions and monotonically
            increasing timestamps.

        Raises:
            ValueError: If ``path`` has fewer than 2 waypoints.
        """
        if len(path) < 2:
            raise ValueError(
                f"path must have at least 2 waypoints, got {len(path)}"
            )

        waypoints = [np.asarray(wp, dtype=np.float64) for wp in path]
        segment_times = self._compute_segment_times(waypoints)
        return self._resample(waypoints, segment_times, start_time)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_config(config: dict[str, Any]) -> dict[str, Any]:
        """Validate and flatten trajectory configuration."""
        src: dict[str, Any] = config.get("trajectory", config)
        require_keys(
            src, _REQUIRED_TRAJECTORY_KEYS, context="trajectory config"
        )
        return dict(src)

    def _compute_segment_times(
        self, waypoints: list[npt.NDArray[np.float64]]
    ) -> list[float]:
        """Compute time for each path segment using trapezoidal profiles.

        For each consecutive waypoint pair, the required segment time is the
        maximum over joints of the trapezoidal duration, lower-bounded by
        ``dt_min``.

        Args:
            waypoints: Waypoint list with at least 2 elements.

        Returns:
            List of segment durations, length ``len(waypoints) - 1``.
        """
        times: list[float] = []
        for i in range(len(waypoints) - 1):
            delta = np.abs(waypoints[i + 1] - waypoints[i])
            t_seg = self._dt_min
            for j in range(self._dof):
                d = float(delta[j])
                v = float(self._v_max[j])
                a = float(self._a_max[j])
                t_j = self._trapezoid_time(d, v, a)
                t_seg = max(t_seg, t_j)
            times.append(t_seg)
        return times

    @staticmethod
    def _trapezoid_time(distance: float, v_max: float, a_max: float) -> float:
        """Compute minimum move time using a trapezoidal velocity profile.

        Chooses between a full trapezoid (accelerate → cruise → decelerate)
        and a triangular profile (accelerate → decelerate) depending on the
        available distance.

        Args:
            distance: Absolute joint displacement (non-negative).
            v_max: Maximum joint velocity.
            a_max: Maximum joint acceleration.

        Returns:
            Minimum time to travel ``distance`` in seconds.  Returns 0.0 for
            zero distance.
        """
        if distance < 1e-12:
            return 0.0
        t_accel = v_max / a_max
        d_full = v_max * t_accel  # distance covered during accel + decel
        if distance >= d_full:
            # Full trapezoidal profile: accel + cruise + decel.
            t_cruise = (distance - d_full) / v_max
            return 2.0 * t_accel + t_cruise
        # Triangular profile: peak velocity is limited.
        return float(2.0 * np.sqrt(distance / a_max))

    def _resample(
        self,
        waypoints: list[npt.NDArray[np.float64]],
        segment_times: list[float],
        start_time: float,
    ) -> TrajectoryResult:
        """Resample the piecewise-linear path at ``control_hz``.

        Uses linear interpolation within each segment.  Velocities are
        computed as finite differences (central differences in the interior,
        forward/backward at the boundaries).

        Args:
            waypoints: Original waypoint list.
            segment_times: Duration of each segment (length N-1).
            start_time: Absolute start time in seconds.

        Returns:
            Fully populated ``TrajectoryResult``.
        """
        dt = 1.0 / self._control_hz
        total_duration = float(np.sum(segment_times))

        # Build cumulative waypoint times.
        cum_times: list[float] = [0.0]
        for t in segment_times:
            cum_times.append(cum_times[-1] + t)

        # Sample times at control rate (always include start and end).
        n_samples = max(
            2, int(np.round(total_duration * self._control_hz)) + 1
        )
        sample_ts = np.linspace(0.0, total_duration, n_samples)

        positions: list[npt.NDArray[np.float64]] = []
        for t in sample_ts:
            positions.append(self._interpolate_at(t, waypoints, cum_times))

        # Compute finite-difference velocities.
        velocities: list[npt.NDArray[np.float64]] = []
        for i, t in enumerate(sample_ts):
            if i == 0:
                seg_dt = float(sample_ts[1] - sample_ts[0])
                vel = (positions[1] - positions[0]) / max(seg_dt, 1e-12)
            elif i == len(sample_ts) - 1:
                seg_dt = float(sample_ts[-1] - sample_ts[-2])
                vel = (positions[-1] - positions[-2]) / max(seg_dt, 1e-12)
            else:
                seg_dt = float(sample_ts[i + 1] - sample_ts[i - 1])
                vel = (positions[i + 1] - positions[i - 1]) / max(
                    seg_dt, 1e-12
                )
            velocities.append(vel)

        timestamps = [start_time + float(t) for t in sample_ts]

        return TrajectoryResult(
            joint_names=list(self._joint_names),
            positions=positions,
            velocities=velocities,
            timestamps=timestamps,
            duration=total_duration,
        )

    def _interpolate_at(
        self,
        t: float,
        waypoints: list[npt.NDArray[np.float64]],
        cum_times: list[float],
    ) -> npt.NDArray[np.float64]:
        """Linearly interpolate the joint configuration at time ``t``.

        Args:
            t: Query time within ``[0, total_duration]``.
            waypoints: Waypoint list.
            cum_times: Cumulative segment times (length == len(waypoints)).

        Returns:
            Interpolated joint configuration, shape ``(DOF,)``.
        """
        total = cum_times[-1]
        t = float(np.clip(t, 0.0, total))

        # Find the enclosing segment.
        for i in range(len(waypoints) - 1):
            t0 = cum_times[i]
            t1 = cum_times[i + 1]
            if t <= t1 + 1e-12:
                seg_duration = t1 - t0
                if seg_duration < 1e-12:
                    return waypoints[i].copy()
                alpha = (t - t0) / seg_duration
                alpha = float(np.clip(alpha, 0.0, 1.0))
                return waypoints[i] + alpha * (waypoints[i + 1] - waypoints[i])

        return waypoints[-1].copy()
