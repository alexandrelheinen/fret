"""PPP prismatic velocity controller (v1.0).

Level-3 pure-Python controller for the PPP gantry.  Uses per-axis
proportional control in joint space (equivalent to Cartesian P-control
because ``q ≡ p_ee``):

    q̇ = Kp · (q_ref − q)   (clipped per joint)

Satisfies requirements FR-CTL-01, FR-CTL-02, FR-CTL-03, FR-CTL-05,
and FR-CTL-06.
"""

from __future__ import annotations

import enum
import pathlib
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

_PPP_DOF: int = 3
_DEFAULT_KP: float = 1.5
_DEFAULT_MAX_VEL: npt.NDArray[np.float64] = np.array(
    [3.0, 3.0, 1.5], dtype=np.float64
)
_DEFAULT_FAULT_THRESHOLD: float = 0.010
_DEFAULT_RATE: float = 50.0
_DEFAULT_TICKS_PER_WAYPOINT: int = 5
_DEFAULT_MAX_ACC: float = 2.0
_DEFAULT_RACE_SPEED: float = 0.9
_DEFAULT_MAX_CARROT_LAG: float = 0.008


class PPPControllerState(enum.IntEnum):
    """FSM states for the PPP prismatic controller."""

    IDLE = 0
    TRACKING = 1
    HALTED = 2


_NodeState = PPPControllerState


class PPPControllerNode:
    """Per-axis P-control for PPP trajectory tracking at 50 Hz.

    Mirrors the public surface of ``ControllerNode`` so ROS wiring can
    dispatch via ``make_controller_node`` in a later PR.

    Args:
        config_path: Path to ``ppp.yml`` or the controllers directory.
    """

    def __init__(self, config_path: str) -> None:
        self._state: _NodeState = _NodeState.IDLE
        self._kp: float = _DEFAULT_KP
        self._max_joint_velocity: npt.NDArray[np.float64] = (
            _DEFAULT_MAX_VEL.copy()
        )
        self._fault_threshold: float = _DEFAULT_FAULT_THRESHOLD
        self._update_rate: float = _DEFAULT_RATE
        self._ticks_per_waypoint: int = _DEFAULT_TICKS_PER_WAYPOINT
        self._max_joint_acc: float = _DEFAULT_MAX_ACC
        self._race_speed: float = _DEFAULT_RACE_SPEED
        self._max_carrot_lag: float = _DEFAULT_MAX_CARROT_LAG
        self._dof: int = _PPP_DOF
        self._current_command: npt.NDArray[np.float64] = np.zeros(
            self._dof, dtype=np.float64
        )
        self._trajectory: list[npt.NDArray[np.float64]] | None = None
        self._trajectory_index: int = 0
        self._tick_count: int = 0
        self._load_config(config_path)

    @property
    def update_rate(self) -> float:
        """Controller update rate [Hz]."""
        return self._update_rate

    @property
    def state(self) -> PPPControllerState:
        """Current FSM state."""
        return self._state

    @property
    def fault_threshold(self) -> float:
        """Joint-space tracking fault threshold [m]."""
        return self._fault_threshold

    @property
    def max_joint_acc(self) -> float:
        """Per-axis joint acceleration limit [m/s²]."""
        return self._max_joint_acc

    @property
    def race_speed(self) -> float:
        """Arc-length carrot advance speed [m/s]."""
        return self._race_speed

    @property
    def max_carrot_lag(self) -> float:
        """Maximum carrot lag before arc-length advance resumes [m]."""
        return self._max_carrot_lag

    def _load_config(self, config_path: str) -> None:
        path = pathlib.Path(config_path)
        if path.is_dir():
            path = path / "ppp.yml"
        if not path.is_file():
            return
        import yaml

        try:
            with path.open() as fh:
                data: Any = yaml.safe_load(fh)
            if not isinstance(data, dict):
                return
            for section in data.values():
                if not isinstance(section, dict):
                    continue
                params: dict[str, Any] = section.get("ros__parameters", {})
                self._kp = float(params.get("kp", self._kp))
                max_vel = params.get(
                    "max_joint_velocity", self._max_joint_velocity
                )
                self._max_joint_velocity = np.asarray(
                    max_vel, dtype=np.float64
                )
                if self._max_joint_velocity.shape != (self._dof,):
                    raise ValueError("max_joint_velocity must have length 3")
                self._fault_threshold = float(
                    params.get("fault_threshold", self._fault_threshold)
                )
                self._update_rate = float(
                    params.get("update_rate", self._update_rate)
                )
                self._ticks_per_waypoint = int(
                    params.get("ticks_per_waypoint", self._ticks_per_waypoint)
                )
                self._max_joint_acc = float(
                    params.get("max_joint_acc", self._max_joint_acc)
                )
                self._race_speed = float(
                    params.get("race_speed", self._race_speed)
                )
                self._max_carrot_lag = float(
                    params.get("max_carrot_lag", self._max_carrot_lag)
                )
                break
        except (yaml.YAMLError, OSError, ValueError, KeyError):
            return

    def set_trajectory(
        self, trajectory: list[npt.NDArray[np.float64]]
    ) -> None:
        """Store a joint trajectory and transition to ``TRACKING``."""
        if len(trajectory) < 2:
            raise ValueError(
                "Trajectory must contain at least 2 waypoints, "
                f"got {len(trajectory)}"
            )
        self._trajectory = [
            np.asarray(q, dtype=np.float64) for q in trajectory
        ]
        self._trajectory_index = 0
        self._tick_count = 0
        self._state = _NodeState.TRACKING

    def has_trajectory(self) -> bool:
        """Return ``True`` when a trajectory has been loaded."""
        return self._trajectory is not None

    def is_trajectory_complete(self) -> bool:
        """Return ``True`` when all trajectory waypoints have been consumed."""
        if self._trajectory is None:
            return False
        return self._trajectory_index >= len(self._trajectory)

    def compute_prismatic_command(
        self,
        kinematics: Kinematics,
        current_positions: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Compute per-axis velocity command for the current trajectory step.

        Args:
            kinematics: PPP kinematics engine (for EE error reporting).
            current_positions: Current joint positions, shape ``(3,)`` [m].

        Returns:
            Joint velocity command [m/s], shape ``(3,)``.
        """
        del kinematics  # PPP EE error equals joint-space position error
        if (
            self._state != _NodeState.TRACKING
            or self._trajectory is None
            or self._trajectory_index >= len(self._trajectory)
        ):
            return np.zeros(self._dof, dtype=np.float64)

        q_ref = self._trajectory[self._trajectory_index]
        joint_error = q_ref - current_positions
        error_m = float(np.linalg.norm(joint_error))

        if self._check_fault(error_m):
            self._enter_halted()
            return self._current_command.copy()

        q_dot = self._kp * joint_error
        q_dot = np.clip(
            q_dot,
            -self._max_joint_velocity,
            self._max_joint_velocity,
        )
        self._current_command = q_dot.astype(np.float64)
        self._tick_count += 1
        if self._tick_count % self._ticks_per_waypoint == 0:
            self._trajectory_index += 1
        return self._current_command.copy()

    def get_ee_error_m(
        self,
        kinematics: Kinematics,
        current_positions: npt.NDArray[np.float64],
    ) -> float:
        """Return EE position error against the current waypoint [m]."""
        if self._trajectory is None or self._trajectory_index >= len(
            self._trajectory
        ):
            return 0.0
        q_ref = self._trajectory[self._trajectory_index]
        x_ref = kinematics.forward_kinematics(q_ref)[:3, 3]
        x_cur = kinematics.forward_kinematics(current_positions)[:3, 3]
        return float(np.linalg.norm(x_ref - x_cur))

    def _check_fault(self, error_m: float) -> bool:
        return error_m > self._fault_threshold

    def _enter_halted(self) -> None:
        self._state = _NodeState.HALTED
        self._current_command = np.zeros(self._dof, dtype=np.float64)

    def _get_current_command(self) -> npt.NDArray[np.float64]:
        """Return a copy of the current joint velocity command."""
        return self._current_command.copy()
