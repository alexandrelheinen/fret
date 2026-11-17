"""Velocity-to-position joint command integrator (FR-CTL-07).

Shared by the SO-101 simulation ``SimIOAdapter`` (PR2) and the real Feetech
bridge (PR3): FRET keeps velocity commands on ``/joint_commands`` (Q3,
decided 2026-11-17), but STS3215 servos are position-only, so something
must convert a commanded joint velocity into a servo position goal. This
module is that converter, independent of MuJoCo or any transport.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class VelocityToPositionIntegrator:
    """Integrates commanded joint velocity into a clamped position goal.

    Args:
        joint_limits: ``(DOF, 2)`` array of ``[lower, upper]`` limits [rad].
        max_joint_velocity: Maximum per-axis velocity magnitude [rad/s]
            (the profile's ``max_joint_velocity``); commanded velocities
            are clamped to ``[-max_joint_velocity, max_joint_velocity]``.
        command_timeout_s: If no new velocity command arrives within this
            many seconds, :meth:`step` holds the last integrated position
            (commands a zero velocity) instead of continuing to move.
        max_dt_s: Upper bound on the elapsed time integrated in a single
            :meth:`step` call, so a scheduler hiccup (a late tick) cannot
            produce a position jump larger than ``max_dt_s *
            max_joint_velocity``.
    """

    def __init__(
        self,
        joint_limits: npt.NDArray[np.float64],
        max_joint_velocity: float,
        command_timeout_s: float,
        max_dt_s: float,
    ) -> None:
        self._limits = np.asarray(joint_limits, dtype=np.float64)
        self._max_joint_velocity = float(max_joint_velocity)
        self._command_timeout_s = float(command_timeout_s)
        self._max_dt_s = float(max_dt_s)

        self._position: npt.NDArray[np.float64] | None = None
        self._last_tick_time: float | None = None
        self._last_command_velocity: npt.NDArray[np.float64] | None = None
        self._last_command_time: float | None = None
        self._rejected_command_count = 0

    @property
    def rejected_command_count(self) -> int:
        """Number of :meth:`command_velocity` calls rejected for bad shape."""
        return self._rejected_command_count

    def start(self, measured_position: npt.NDArray[np.float64]) -> None:
        """Seed the integrator from the measured joint position.

        Must be called once before the first :meth:`step` (FR-CTL-07:
        "start from the measured joint position", not an arbitrary zero).
        """
        q0 = np.asarray(measured_position, dtype=np.float64)
        self._position = np.clip(q0, self._limits[:, 0], self._limits[:, 1])
        self._last_tick_time = None
        self._last_command_velocity = None
        self._last_command_time = None

    def command_velocity(
        self, velocity: npt.NDArray[np.float64], now_s: float
    ) -> None:
        """Record a commanded joint velocity, timestamped ``now_s``.

        A command whose length does not match the integrator's DOF is
        rejected (the last good command, if any, keeps being used) and
        counted in :attr:`rejected_command_count`; it never raises, so a
        single malformed message cannot stop the control loop.
        """
        if self._position is None:
            raise RuntimeError(
                "start() must be called before command_velocity()"
            )
        v = np.asarray(velocity, dtype=np.float64)
        if v.shape != self._position.shape:
            self._rejected_command_count += 1
            return
        self._last_command_velocity = v
        self._last_command_time = float(now_s)

    def step(self, now_s: float) -> npt.NDArray[np.float64]:
        """Advance the integrator to ``now_s`` and return the new position.

        Raises:
            RuntimeError: If :meth:`start` has not been called yet.
        """
        if self._position is None:
            raise RuntimeError("start() must be called before step()")

        if self._last_tick_time is None:
            dt = 0.0
        else:
            dt = min(now_s - self._last_tick_time, self._max_dt_s)
        self._last_tick_time = float(now_s)

        timed_out = (
            self._last_command_time is None
            or (now_s - self._last_command_time) > self._command_timeout_s
        )
        if timed_out or self._last_command_velocity is None:
            velocity = np.zeros_like(self._position)
        else:
            velocity = np.clip(
                self._last_command_velocity,
                -self._max_joint_velocity,
                self._max_joint_velocity,
            )

        self._position = np.clip(
            self._position + velocity * dt,
            self._limits[:, 0],
            self._limits[:, 1],
        )
        return self._position.copy()
