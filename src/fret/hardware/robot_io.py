"""Hardware ``RobotIO`` adapter for the pick-place FSM (no MuJoCo)."""

from __future__ import annotations

import time
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.hardware.bridge_core import BridgeCore, BridgeState


class HardwareRobotIO:
    """Drive a :class:`BridgeCore` through the ``RobotIO`` protocol."""

    def __init__(
        self,
        core: BridgeCore,
        *,
        gripper_index: int = 5,
        clock: Any | None = None,
    ) -> None:
        self._core = core
        self._gripper_index = gripper_index
        self._clock = clock or time.monotonic
        self._pending = np.zeros(core.positions.shape, dtype=np.float64)

    def read_joint_state(self) -> npt.NDArray[np.float64]:
        if self._core.state == BridgeState.DISCONNECTED:
            self._core.connect()
        self._core.cycle(self._clock())
        return self._core.positions

    def send_joint_velocity(
        self, joint_velocity: npt.NDArray[np.float64]
    ) -> None:
        v = np.asarray(joint_velocity, dtype=np.float64)
        if v.shape != self._pending.shape:
            # Allow 5-DOF arm commands by holding gripper velocity.
            if v.shape == (self._pending.shape[0] - 1,):
                full = self._pending.copy()
                full[:-1] = v
                v = full
            else:
                raise ValueError(
                    f"joint_velocity shape {v.shape} != {self._pending.shape}"
                )
        self._pending = v
        self._core.command_velocity(v, self._clock())
        if self._core.state == BridgeState.CONNECTED_DISABLED:
            # FSM may command before enable; stay disabled (PR4 soft-start).
            self._core.cycle(self._clock())
            return
        if self._core.state == BridgeState.ENABLED:
            self._core.cycle(self._clock())

    def set_gripper(self, value: float) -> None:
        """Map gripper open fraction [0, 1] to a joint velocity nudge."""
        # Hold other joints; drive gripper toward open (positive) / close.
        cmd = np.zeros_like(self._pending)
        cmd[self._gripper_index] = float(np.clip(value, -1.0, 1.0))
        self.send_joint_velocity(cmd)

    def now(self) -> float:
        return float(self._clock())
