"""Jacobian-based trajectory tracking controller (Level 3 logic core).

Implements the state machine, fault detection, and joint-command management
for the 50 Hz Jacobian-based trajectory tracking controller.  ROS 2 node
wiring (``super().__init__``, timer, subscription, and topic publishing) is
deferred to the Level 4 implementation.

FSM states: ``IDLE → TRACKING → HALTED`` — see docs/interfaces.md.

Publishes (Level 4):
    /joint_commands   (``trajectory_msgs/JointTrajectory``)
    /controller_fault (``std_msgs/Bool``) — True when EE error > fault_threshold.

Satisfies requirements FR-CTL-01 through FR-CTL-06.
"""

from __future__ import annotations

import enum
import pathlib
from typing import Any

import numpy as np
import numpy.typing as npt

# ---------------------------------------------------------------------------
# Controller FSM
# ---------------------------------------------------------------------------


class _NodeState(enum.IntEnum):
    """Internal FSM state for the controller."""

    IDLE = 0
    TRACKING = 1
    HALTED = 2


# ---------------------------------------------------------------------------
# Default configuration values (mirrors config/controllers/jacobian.yml)
# ---------------------------------------------------------------------------

_SCARA_DOF: int = 3
_DEFAULT_FAULT_THRESHOLD: float = 0.020  # [m]
_DEFAULT_DAMPING: float = 0.01
_DEFAULT_MAX_VEL: float = 1.57  # [rad/s]
_DEFAULT_RATE: float = 50.0  # [Hz]


class ControllerNode:
    """Level 3 logic core for Jacobian-based 50 Hz trajectory tracking.

    This class encapsulates the pure-Python state machine and algorithmic
    methods.  It is intentionally decoupled from the ROS 2 Node base class
    so that the logic can be unit-tested without a live ROS context.  The
    Level 4 implementation will subclass ``rclpy.node.Node`` and wire in
    timers, subscriptions, and publishers.

    Args:
        model: Robot model name.  Accepts ``"scara"``; reserved for future
            models.
        config_path: Path to the controller YAML configuration file
            (e.g. ``config/controllers/jacobian.yml``) or a directory.  If
            the file cannot be found or parsed, default values are used
            silently.
    """

    def __init__(self, model: str, config_path: str) -> None:
        self._model = model
        self._state: _NodeState = _NodeState.IDLE
        self._fault_threshold: float = _DEFAULT_FAULT_THRESHOLD
        self._damping: float = _DEFAULT_DAMPING
        self._max_joint_velocity: float = _DEFAULT_MAX_VEL
        self._update_rate: float = _DEFAULT_RATE
        self._dof: int = _SCARA_DOF
        self._current_command: npt.NDArray[np.float64] = np.zeros(
            self._dof, dtype=np.float64
        )
        self._trajectory: list[npt.NDArray[np.float64]] | None = None
        self._trajectory_index: int = 0
        self._load_config(config_path)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def _load_config(self, config_path: str) -> None:
        """Load controller parameters from a YAML file if one is found.

        Silently falls back to built-in defaults on any error.

        Args:
            config_path: Path to the YAML file or a directory.  If a
                directory is given, ``jacobian.yml`` is tried inside it.
        """
        import yaml  # type: ignore[import-untyped]

        path = pathlib.Path(config_path)
        if path.is_dir():
            path = path / "jacobian.yml"
        if not path.is_file():
            return
        try:
            with path.open() as fh:
                data: Any = yaml.safe_load(fh)
            if not isinstance(data, dict):
                return
            # ROS 2 parameter YAML layout: /**:  ros__parameters:  key: val
            for section in data.values():
                if not isinstance(section, dict):
                    continue
                params: dict[str, Any] = section.get("ros__parameters", {})
                self._fault_threshold = float(
                    params.get("fault_threshold", self._fault_threshold)
                )
                self._damping = float(
                    params.get("damping_factor", self._damping)
                )
                self._max_joint_velocity = float(
                    params.get("max_joint_velocity", self._max_joint_velocity)
                )
                self._update_rate = float(
                    params.get("update_rate", self._update_rate)
                )
                break
        except (yaml.YAMLError, OSError, ValueError, KeyError) as exc:
            # Silently fall back to defaults; config errors are non-fatal
            _ = exc

    # ------------------------------------------------------------------
    # State machine helpers (unit-testable)
    # ------------------------------------------------------------------

    def _check_fault(self, error_m: float) -> bool:
        """Return ``True`` if the EE tracking error exceeds the threshold.

        Args:
            error_m: End-effector position error in metres.

        Returns:
            ``True`` when ``error_m > fault_threshold`` (FR-CTL-04).
        """
        return error_m > self._fault_threshold

    def _enter_halted(self) -> None:
        """Transition to the ``HALTED`` state and zero all joint commands."""
        self._state = _NodeState.HALTED
        self._current_command = np.zeros(self._dof, dtype=np.float64)

    def _get_current_command(self) -> npt.NDArray[np.float64]:
        """Return a copy of the current joint velocity command.

        Returns:
            Joint velocity command array, shape ``(DOF,)``.
        """
        return self._current_command.copy()
