"""SO-101 MuJoCo position-actuator IO with shared SafetySupervisor (FR-HW-69).

Converts velocity commands via :class:`VelocityToPositionIntegrator`, then
filters goals through the same :class:`~fret.hardware.safety.SafetySupervisor`
the hardware bridge uses before writing MuJoCo ``data.ctrl``.
"""

from __future__ import annotations

from typing import Any, Callable, Sequence

import numpy as np
import numpy.typing as npt

from fret.control.velocity_to_position import VelocityToPositionIntegrator
from fret.hardware.safety import SafetyConfig, SafetySupervisor


class PositionActuatorIO:
    """Sim twin of the Feetech bridge write path (integrator + supervisor)."""

    def __init__(
        self,
        model: Any,
        data: Any,
        *,
        actuator_ids: Sequence[int],
        joint_limits_rad: npt.NDArray[np.float64],
        max_joint_velocity: float = 2.0,
        command_timeout_s: float = 0.2,
        max_dt_s: float = 0.05,
        safety_config: SafetyConfig | None = None,
        clock: Callable[[], float] | None = None,
        mj: Any | None = None,
    ) -> None:
        if len(actuator_ids) == 0:
            raise ValueError("actuator_ids must be non-empty")
        self._model = model
        self._data = data
        self._actuator_ids = tuple(int(a) for a in actuator_ids)
        self._dof = len(self._actuator_ids)
        self._mj = mj
        self._clock = clock or (lambda: float(data.time))
        cfg = safety_config or SafetyConfig(
            max_joint_velocity=max_joint_velocity,
            dt_s=min(max_dt_s, 0.02),
        )
        self._safety = SafetySupervisor(
            np.asarray(joint_limits_rad, dtype=np.float64),
            config=cfg,
        )
        self._integrator = VelocityToPositionIntegrator(
            joint_limits=np.asarray(joint_limits_rad, dtype=np.float64),
            max_joint_velocity=max_joint_velocity,
            command_timeout_s=command_timeout_s,
            max_dt_s=max_dt_s,
        )
        measured = self.read_positions()
        self._integrator.start(measured)
        self._safety.soft_start_goals(measured)
        self._safety.set_enabled(True)
        self._safety.renew_lease(self._clock())
        self._safety.note_command(self._clock())

    @property
    def safety(self) -> SafetySupervisor:
        return self._safety

    def read_positions(self) -> npt.NDArray[np.float64]:
        """Read measured joint positions from actuator-linked joints."""
        out = np.zeros(self._dof, dtype=np.float64)
        for i, aid in enumerate(self._actuator_ids):
            # Prefer qpos via actuator_trnid when available.
            try:
                jnt_id = int(self._model.actuator_trnid[aid][0])
                qpos_adr = int(self._model.jnt_qposadr[jnt_id])
                out[i] = float(self._data.qpos[qpos_adr])
            except Exception:  # noqa: BLE001
                out[i] = float(self._data.ctrl[aid])
        return out

    def command_velocity(
        self, velocity: npt.NDArray[np.float64], now_s: float | None = None
    ) -> None:
        t = self._clock() if now_s is None else now_s
        self._integrator.command_velocity(
            np.asarray(velocity, dtype=np.float64), t
        )
        self._safety.note_command(t)

    def renew_lease(self, now_s: float | None = None) -> None:
        self._safety.renew_lease(self._clock() if now_s is None else now_s)

    def latch_stop(self) -> None:
        self._safety.latch_stop()

    def clear_stop(self) -> None:
        self._safety.clear_stop()

    def step(self, now_s: float | None = None) -> npt.NDArray[np.float64]:
        """Integrate, filter, and write ctrl. Returns written goals."""
        t = self._clock() if now_s is None else now_s
        measured = self.read_positions()
        goals = self._integrator.step(t)
        filtered = self._safety.filter_goals(goals, measured, t)
        for i, aid in enumerate(self._actuator_ids):
            self._data.ctrl[aid] = float(filtered[i])
        return filtered
