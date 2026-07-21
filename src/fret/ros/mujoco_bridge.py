"""MuJoCo backend adapter for FRET simulator I/O (v1.0 + v1.2 physics).

Level-3 ``MuJoCoBridgeCore`` integrates joint velocity commands against an
MJCF scene without requiring a live ROS context.  When the optional
``mujoco`` package is installed the core also keeps an ``MjModel`` /
``MjData`` pair in sync for forward kinematics and future rendering hooks.

In **physics mode** (v1.2), velocity commands drive MuJoCo actuators and
``mj_step`` advances the simulation; joint state is read from ``qpos`` /
``qvel`` (FR-SIM-07).

Level-4 ``MuJoCoBridgeNode`` subscribes to ``/joint_commands`` and publishes
``/joint_states`` at the configured rate (default 50 Hz).

Satisfies requirements FR-SIM-01, FR-SIM-02, and FR-SIM-09.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.ros.mujoco_physics_log import (
    ContactLogConfig,
    PhysicsContactLogger,
    agent_obstacle_contact_forces,
    contact_log_config_from_bridge_yaml,
)
from fret.simulation.turtlebot3_unit import (
    body_velocity_to_wheel_rates,
    clip_wheel_rates,
)

# Dubins race workspace limits for compact dubins_race.xml (10 m lab).
_DUBINS_RACE_LIMITS: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 10.0],
        [0.0, 10.0],
        [-np.pi, np.pi],
    ],
    dtype=np.float64,
)

# Freejoint bases for the three TurtleBot3 race agents (7-DoF qpos each).
_RRT_BASE_JOINT: str = "rrt_base_joint"
_SST_BASE_JOINT: str = "sst_base_joint"
_DUMMY_BASE_JOINT: str = "dummy_base_joint"
_DUBINS_BASE_JOINTS: tuple[str, str, str] = (
    _RRT_BASE_JOINT,
    _SST_BASE_JOINT,
    _DUMMY_BASE_JOINT,
)

# Collision shell geoms used by the real-contact collision monitor.
_RRT_COLLISION_GEOM: str = "rrt_collision"
_SST_COLLISION_GEOM: str = "sst_collision"
_DUMMY_COLLISION_GEOM: str = "dummy_collision"
_DUBINS_COLLISION_GEOMS: tuple[str, str, str] = (
    _RRT_COLLISION_GEOM,
    _SST_COLLISION_GEOM,
    _DUMMY_COLLISION_GEOM,
)

# Match TurtleBot3 Burger Menagerie geometry; race SITL allows 2× nominal
# Menagerie wheel rate so showcase cruise (0.36 m/s) is reachable.
_AGENT_BASE_Z_M: float = 0.033
_WHEEL_RADIUS_M: float = 0.033
_TRACK_WIDTH_M: float = 0.16
_WHEEL_CTRL_LIMIT_RAD_S: float = 13.34

_DEFAULT_UPDATE_RATE_HZ: float = 50.0
_DEFAULT_MJCF_TIMESTEP_S: float = 0.002
_DEFAULT_SUBSTEPS_PER_TICK: int = 25


def _yaw_to_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Planar yaw (about +z) to MuJoCo quaternion ``(w, x, y, z)``."""
    half = 0.5 * float(yaw)
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _quat_wxyz_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract planar yaw from a MuJoCo quaternion ``(w, x, y, z)``."""
    sin_y = 2.0 * (qw * qz + qx * qy)
    cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_y, cos_y)


def _world_velocity_to_body_forward(
    vx: float,
    vy: float,
    yaw: float,
) -> float:
    """Project world-frame ``(vx, vy)`` onto the body forward axis."""
    return float(vx * math.cos(yaw) + vy * math.sin(yaw))


@dataclass(frozen=True)
class ActuatorTable:
    """Runtime actuator gain table loaded from ``simulation/mujoco.yml``."""

    names: tuple[str, ...]
    kv: tuple[float, ...]
    forcerange: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class PhysicsBridgeConfig:
    """Physics SITL settings for MuJoCo bridge cores (v1.2)."""

    physics_mode: bool
    substeps_per_tick: int
    actuators: ActuatorTable | None


def _parse_bool(value: Any) -> bool:
    """Parse ROS / YAML boolean parameters."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "on"}


def _require_actuator_table(cfg: dict[str, Any], model: str) -> ActuatorTable:
    """Load and validate the actuator table for ``model``."""
    actuators = cfg.get("actuators")
    if not isinstance(actuators, dict):
        raise ValueError("physics_mode requires 'actuators' in mujoco.yml")

    section = actuators.get(model)
    if not isinstance(section, dict):
        raise ValueError(
            f"physics_mode requires actuators.{model} in mujoco.yml"
        )

    names_raw = section.get("names")
    kv_raw = section.get("kv")
    force_raw = section.get("forcerange")
    if not isinstance(names_raw, list) or not names_raw:
        raise ValueError(f"actuators.{model}.names must be a non-empty list")
    if not isinstance(kv_raw, list) or len(kv_raw) != len(names_raw):
        raise ValueError(
            f"actuators.{model}.kv must match names length "
            f"({len(names_raw)})"
        )
    if not isinstance(force_raw, list) or len(force_raw) != len(names_raw):
        raise ValueError(
            f"actuators.{model}.forcerange must match names length "
            f"({len(names_raw)})"
        )

    forcerange: list[tuple[float, float]] = []
    for entry in force_raw:
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            raise ValueError(
                f"actuators.{model}.forcerange entries must be [min, max]"
            )
        forcerange.append((float(entry[0]), float(entry[1])))

    return ActuatorTable(
        names=tuple(str(name) for name in names_raw),
        kv=tuple(float(v) for v in kv_raw),
        forcerange=tuple(forcerange),
    )


def physics_config_from_bridge_yaml(
    cfg: dict[str, Any],
    model: str,
    *,
    physics_mode: bool | None = None,
) -> PhysicsBridgeConfig:
    """Build :class:`PhysicsBridgeConfig` from bridge YAML parameters."""
    mode = (
        _parse_bool(physics_mode)
        if physics_mode is not None
        else _parse_bool(cfg.get("physics_mode", False))
    )
    substeps = int(cfg.get("substeps_per_tick", _DEFAULT_SUBSTEPS_PER_TICK))
    if substeps <= 0:
        raise ValueError("substeps_per_tick must be positive")

    actuators = _require_actuator_table(cfg, model) if mode else None
    return PhysicsBridgeConfig(
        physics_mode=mode,
        substeps_per_tick=substeps,
        actuators=actuators,
    )


def _bind_joint_addresses(
    model: Any,
    mujoco: Any,
    joint_names: list[str],
) -> tuple[list[int], list[int]]:
    """Return ``(qpos_adrs, qvel_adrs)`` for ordered joint names."""
    qpos_adrs: list[int] = []
    qvel_adrs: list[int] = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )
        if joint_id < 0:
            raise ValueError(f"Joint not found in MJCF: {name}")
        qpos_adrs.append(int(model.jnt_qposadr[joint_id]))
        qvel_adrs.append(int(model.jnt_dofadr[joint_id]))
    return qpos_adrs, qvel_adrs


def _bind_actuator_ids(
    model: Any,
    mujoco: Any,
    actuator_names: tuple[str, ...],
) -> list[int]:
    """Return MuJoCo actuator ids in command order."""
    actuator_ids: list[int] = []
    for name in actuator_names:
        act_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            name,
        )
        if act_id < 0:
            raise ValueError(f"Actuator not found in MJCF: {name}")
        actuator_ids.append(int(act_id))
    return actuator_ids


def _apply_actuator_gains(model: Any, table: ActuatorTable) -> None:
    """Apply YAML actuator gains to an ``MjModel``."""
    if model.nu != len(table.names):
        raise ValueError(
            f"MJCF has {model.nu} actuators; expected {len(table.names)}"
        )
    for idx, (kv, (fmin, fmax)) in enumerate(
        zip(table.kv, table.forcerange, strict=True)
    ):
        model.actuator_gainprm[idx, 0] = float(kv)
        model.actuator_forcerange[idx, 0] = float(fmin)
        model.actuator_forcerange[idx, 1] = float(fmax)


def _project_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def resolve_mjcf_path(
    model: str,
    scenario: str,
    mjcf_override: str | pathlib.Path | None = None,
) -> pathlib.Path:
    """Return the MJCF file path for a model/scenario pair.

    Args:
        model: Robot model name (e.g. ``dubins``).
        scenario: Scenario stem (e.g. ``dubins_race``).
        mjcf_override: Optional explicit MJCF path.

    Returns:
        Resolved path to an existing MJCF file.

    Raises:
        FileNotFoundError: If an override path does not exist.
        ValueError: If the model/scenario combination is unsupported.
    """
    if mjcf_override is not None:
        path = pathlib.Path(mjcf_override)
        if not path.is_file():
            raise FileNotFoundError(f"MJCF not found: {path}")
        return path

    from fret.sitl_config import mjcf_path as resolve_installed_mjcf

    return resolve_installed_mjcf(model, scenario)


def integrate_joint_velocities(
    positions: npt.NDArray[np.float64],
    velocities: npt.NDArray[np.float64],
    dt: float,
    limits: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """Integrate joint velocities and clip to per-axis limits.

    Args:
        positions: Current joint positions, shape ``(DOF,)``.
        velocities: Commanded joint velocities [m/s or rad/s], shape ``(DOF,)``.
        dt: Integration timestep [s].
        limits: Joint limits array, shape ``(DOF, 2)`` with ``[lower, upper]``.

    Returns:
        Updated joint positions, shape ``(DOF,)``.
    """
    if positions.shape != velocities.shape:
        raise ValueError("positions and velocities must have the same shape")
    if limits.shape != (positions.shape[0], 2):
        raise ValueError(
            f"limits must have shape ({positions.shape[0]}, 2), got {limits.shape}"
        )
    updated = positions + velocities * dt
    return np.clip(updated, limits[:, 0], limits[:, 1]).astype(np.float64)


def make_mujoco_bridge_core(
    model: str,
    scenario: str,
    *,
    mjcf_path: str | pathlib.Path | None = None,
    initial_positions: npt.NDArray[np.float64] | None = None,
    physics_config: PhysicsBridgeConfig | None = None,
) -> MuJoCoBridgeCore:
    """Build a model-appropriate MuJoCo bridge core (FR-SYS-01).

    Args:
        model: Robot model name.
        scenario: Scenario stem used for MJCF resolution.
        mjcf_path: Optional explicit MJCF path override.
        initial_positions: Optional initial joint configuration.
        physics_config: Optional physics SITL settings (v1.2).

    Returns:
        Configured ``MuJoCoBridgeCore`` instance.

    Raises:
        ValueError: If ``model`` is not recognised.
    """
    _ = (scenario, mjcf_path, initial_positions, physics_config)
    if model == "dubins":
        raise ValueError(
            "Use make_dubins_race_bridge_core() for the dual-agent dubins race."
        )

    raise ValueError(f"Unknown MuJoCo bridge model: {model!r}")


class MuJoCoBridgeCore:
    """Level-3 MuJoCo joint I/O core.

      Integrates velocity commands in joint space and optionally mirrors the
    state into a MuJoCo model when the package is available.

      Args:
          mjcf_path: Path to the MJCF scene file.
          joint_names: Ordered joint names matching command/state vectors.
          limits: Joint limits array, shape ``(DOF, 2)``.
          initial_positions: Starting joint configuration.
    """

    def __init__(
        self,
        mjcf_path: str | pathlib.Path,
        joint_names: list[str],
        limits: npt.NDArray[np.float64],
        initial_positions: npt.NDArray[np.float64],
    ) -> None:
        self._mjcf_path = pathlib.Path(mjcf_path)
        self._joint_names = list(joint_names)
        self._limits = np.asarray(limits, dtype=np.float64)
        self._positions = np.clip(
            np.asarray(initial_positions, dtype=np.float64),
            self._limits[:, 0],
            self._limits[:, 1],
        )
        self._velocities = np.zeros_like(self._positions)
        self._physics_mode = False
        self._substeps_per_tick = _DEFAULT_SUBSTEPS_PER_TICK
        self._actuator_ids: list[int] = []
        self._mujoco: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._qpos_adrs: list[int] = []
        self._qvel_adrs: list[int] = []
        self._contact_logger: PhysicsContactLogger | None = None
        self._load_mujoco_optional()
        self._seed_mujoco_state()

    @property
    def physics_mode(self) -> bool:
        """Return ``True`` when actuator-driven ``mj_step`` is active."""
        return self._physics_mode

    def configure_physics(self, config: PhysicsBridgeConfig) -> None:
        """Enable or disable physics SITL mode (v1.2)."""
        self._physics_mode = bool(config.physics_mode)
        self._substeps_per_tick = int(config.substeps_per_tick)
        if not self._physics_mode:
            return
        if self._model is None or self._mujoco is None:
            raise RuntimeError(
                "physics_mode requires the optional mujoco package"
            )
        if config.actuators is None:
            raise ValueError("physics_mode requires actuator configuration")
        self._actuator_ids = _bind_actuator_ids(
            self._model,
            self._mujoco,
            config.actuators.names,
        )
        _apply_actuator_gains(self._model, config.actuators)

    def configure_contact_logging(self, config: ContactLogConfig) -> None:
        """Enable JSONL contact logging for physics SITL ticks (T12-05)."""
        if not config.enabled:
            self._contact_logger = None
            return
        self._contact_logger = PhysicsContactLogger(config)

    @property
    def contact_logger(self) -> PhysicsContactLogger | None:
        """Return the active contact logger, if any."""
        return self._contact_logger

    def finalize_physics_metrics(
        self,
        *,
        max_tracking_error_m: float | None = None,
    ) -> pathlib.Path | None:
        """Flush contact logs and write shutdown metrics."""
        if self._contact_logger is None:
            return None
        path = self._contact_logger.close(
            max_tracking_error_m=max_tracking_error_m
        )
        self._contact_logger = None
        return path

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names."""
        return list(self._joint_names)

    @property
    def limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)``."""
        return self._limits.copy()

    @property
    def mjcf_path(self) -> pathlib.Path:
        """Loaded MJCF scene path."""
        return self._mjcf_path

    @property
    def has_mujoco_runtime(self) -> bool:
        """Return ``True`` when the optional MuJoCo package is active."""
        return self._model is not None

    def get_positions(self) -> npt.NDArray[np.float64]:
        """Return a copy of the current joint positions."""
        return self._positions.copy()

    def get_velocities(self) -> npt.NDArray[np.float64]:
        """Return a copy of the most recent joint velocities."""
        return self._velocities.copy()

    def snapshot_qpos(self) -> npt.NDArray[np.float64]:
        """Return a copy of the full MuJoCo ``qpos`` vector for render replay."""
        if self._data is None:
            raise RuntimeError("snapshot_qpos() requires the mujoco package")
        return np.array(self._data.qpos, dtype=np.float64, copy=True)

    def set_positions(self, positions: npt.NDArray[np.float64]) -> None:
        """Set joint positions directly (clipped to limits).

        Args:
            positions: Joint configuration, shape ``(DOF,)``.

        Raises:
            RuntimeError: When ``physics_mode`` is active (FR-SIM-07).
        """
        if self._physics_mode:
            raise RuntimeError(
                "set_positions() is forbidden while physics_mode is active"
            )
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != self._positions.shape:
            raise ValueError(
                f"Expected shape {self._positions.shape}, got {q.shape}"
            )
        self._positions = np.clip(q, self._limits[:, 0], self._limits[:, 1])
        self._write_mujoco_state()

    def step_physics(
        self,
        velocities: npt.NDArray[np.float64],
        *,
        substeps: int | None = None,
    ) -> npt.NDArray[np.float64]:
        """Advance the MuJoCo simulation with velocity actuator commands.

        Args:
            velocities: Target joint velocities, shape ``(DOF,)``.
            substeps: Optional override for ``mj_step`` count per tick.

        Returns:
            Simulated joint positions read from ``qpos``.

        Raises:
            RuntimeError: When MuJoCo is unavailable or physics is disabled.
        """
        if not self._physics_mode:
            raise RuntimeError("step_physics() requires physics_mode=True")
        if self._model is None or self._data is None or self._mujoco is None:
            raise RuntimeError("step_physics() requires the mujoco package")

        v = np.asarray(velocities, dtype=np.float64)
        if v.shape != self._positions.shape:
            raise ValueError(
                f"Expected velocity shape {self._positions.shape}, got {v.shape}"
            )
        if len(self._actuator_ids) != v.shape[0]:
            raise RuntimeError("Actuator count does not match DOF")

        self._velocities = v.copy()
        for idx, act_id in enumerate(self._actuator_ids):
            self._data.ctrl[act_id] = float(v[idx])

        step_count = (
            self._substeps_per_tick if substeps is None else int(substeps)
        )
        if step_count <= 0:
            raise ValueError("substeps must be positive")

        for _ in range(step_count):
            self._mujoco.mj_step(self._model, self._data)

        if self._contact_logger is not None:
            self._contact_logger.record_tick(
                self._model,
                self._data,
                self._mujoco,
            )

        self._read_state_from_mujoco()
        return self._positions.copy()

    def step(
        self,
        velocities: npt.NDArray[np.float64],
        dt: float,
    ) -> npt.NDArray[np.float64]:
        """Integrate one control timestep from a velocity command.

        Args:
            velocities: Commanded joint velocities, shape ``(DOF,)``.
            dt: Integration period [s].

        Returns:
            Updated joint positions after integration.
        """
        if self._physics_mode:
            return self.step_physics(velocities)
        v = np.asarray(velocities, dtype=np.float64)
        if v.shape != self._positions.shape:
            raise ValueError(
                f"Expected velocity shape {self._positions.shape}, got {v.shape}"
            )
        self._velocities = v.copy()
        self._positions = integrate_joint_velocities(
            self._positions,
            self._velocities,
            dt,
            self._limits,
        )
        self._write_mujoco_state()
        return self._positions.copy()

    def _load_mujoco_optional(self) -> None:
        try:
            import mujoco
        except ImportError:
            return

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(
            str(self._mjcf_path.resolve())
        )
        self._data = mujoco.MjData(self._model)
        self._qpos_adrs, self._qvel_adrs = _bind_joint_addresses(
            self._model,
            mujoco,
            self._joint_names,
        )

    def _seed_mujoco_state(self) -> None:
        """Write initial ``qpos`` once at construction (before physics mode)."""
        if self._model is None or self._data is None or self._mujoco is None:
            return
        for idx, adr in enumerate(self._qpos_adrs):
            self._data.qpos[adr] = float(self._positions[idx])
        self._mujoco.mj_forward(self._model, self._data)

    def _read_state_from_mujoco(self) -> None:
        if self._model is None or self._data is None:
            return
        positions = np.empty(len(self._qpos_adrs), dtype=np.float64)
        velocities = np.empty(len(self._qvel_adrs), dtype=np.float64)
        for idx, (qpos_adr, qvel_adr) in enumerate(
            zip(self._qpos_adrs, self._qvel_adrs, strict=True)
        ):
            positions[idx] = float(self._data.qpos[qpos_adr])
            velocities[idx] = float(self._data.qvel[qvel_adr])
        self._positions = np.clip(
            positions,
            self._limits[:, 0],
            self._limits[:, 1],
        )
        self._velocities = velocities

    def _write_mujoco_state(self) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            return
        if self._physics_mode:
            raise RuntimeError(
                "pose injection is forbidden while physics_mode is active"
            )
        for idx, adr in enumerate(self._qpos_adrs):
            self._data.qpos[adr] = float(self._positions[idx])
        self._mujoco.mj_forward(self._model, self._data)


class DubinsRaceBridgeCore:
    """MuJoCo state mirror for the triple-agent Dubins race scene.

    In kinematic mode, writes RRT*, SST, and dummy poses into
    ``dubins_race.xml`` freejoint coordinates (planar ``x, y, yaw``) without
    integrating dynamics.

    In physics mode (v1.2), six wheel velocity actuators are driven via
    :meth:`step_physics`. The race runner still supplies a length-9
    world-frame command ``(vx, vy, ω)×3``; each agent's command is converted
    to body forward speed and yaw rate, then to left/right wheel rates.
    Poses are read from freejoint ``qpos``. Non-holonomy comes from wheel
    contact only (pure ``mj_step``, no SE(2) qvel projection).
    """

    def __init__(
        self,
        *,
        mjcf_path: str | pathlib.Path | None = None,
        initial_rrt: npt.NDArray[np.float64] | None = None,
        initial_sst: npt.NDArray[np.float64] | None = None,
        initial_dummy: npt.NDArray[np.float64] | None = None,
        physics_config: PhysicsBridgeConfig | None = None,
    ) -> None:
        resolved = resolve_mjcf_path("dubins", "dubins_race", mjcf_path)
        self._mjcf_path = resolved
        self._limits = _DUBINS_RACE_LIMITS
        self._rrt = (
            np.zeros(3, dtype=np.float64)
            if initial_rrt is None
            else np.clip(
                np.asarray(initial_rrt, dtype=np.float64),
                self._limits[:, 0],
                self._limits[:, 1],
            )
        )
        self._sst = (
            np.zeros(3, dtype=np.float64)
            if initial_sst is None
            else np.clip(
                np.asarray(initial_sst, dtype=np.float64),
                self._limits[:, 0],
                self._limits[:, 1],
            )
        )
        self._dummy = (
            np.zeros(3, dtype=np.float64)
            if initial_dummy is None
            else np.clip(
                np.asarray(initial_dummy, dtype=np.float64),
                self._limits[:, 0],
                self._limits[:, 1],
            )
        )
        self._mujoco: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._qpos_adrs: dict[str, int] = {}
        self._physics_mode = False
        self._substeps_per_tick = _DEFAULT_SUBSTEPS_PER_TICK
        self._actuator_ids: list[int] = []
        self._velocities = np.zeros(9, dtype=np.float64)
        self._contact_logger: PhysicsContactLogger | None = None
        self._collision_forces_n: dict[str, float] = dict.fromkeys(
            _DUBINS_COLLISION_GEOMS, 0.0
        )
        self._load_mujoco_optional()
        self._seed_mujoco_state()
        if physics_config is not None:
            self.configure_physics(physics_config)

    @property
    def physics_mode(self) -> bool:
        """Return ``True`` when actuator-driven ``mj_step`` is active."""
        return self._physics_mode

    def configure_physics(self, config: PhysicsBridgeConfig) -> None:
        """Enable or disable physics SITL mode (v1.2)."""
        self._physics_mode = bool(config.physics_mode)
        self._substeps_per_tick = int(config.substeps_per_tick)
        if not self._physics_mode:
            return
        if self._model is None or self._mujoco is None:
            raise RuntimeError(
                "physics_mode requires the optional mujoco package"
            )
        if config.actuators is None:
            raise ValueError("physics_mode requires actuator configuration")
        self._actuator_ids = _bind_actuator_ids(
            self._model,
            self._mujoco,
            config.actuators.names,
        )
        if len(self._actuator_ids) != 6:
            raise RuntimeError(
                "Dubins race MJCF must expose six wheel actuators"
            )
        _apply_actuator_gains(self._model, config.actuators)

    def configure_contact_logging(self, config: ContactLogConfig) -> None:
        """Enable JSONL contact logging for physics SITL ticks (T12-05)."""
        if not config.enabled:
            self._contact_logger = None
            return
        self._contact_logger = PhysicsContactLogger(config)

    @property
    def contact_logger(self) -> PhysicsContactLogger | None:
        """Return the active contact logger, if any."""
        return self._contact_logger

    def finalize_physics_metrics(
        self,
        *,
        max_tracking_error_m: float | None = None,
    ) -> pathlib.Path | None:
        """Flush contact logs and write shutdown metrics."""
        if self._contact_logger is None:
            return None
        path = self._contact_logger.close(
            max_tracking_error_m=max_tracking_error_m
        )
        self._contact_logger = None
        return path

    @property
    def mjcf_path(self) -> pathlib.Path:
        """Loaded MJCF scene path."""
        return self._mjcf_path

    @property
    def has_mujoco_runtime(self) -> bool:
        """Return ``True`` when MuJoCo is available."""
        return self._model is not None

    def set_rrt_pose(self, pose: tuple[float, float, float]) -> None:
        """Update RRT* agent pose ``(x, y, heading)``."""
        if self._physics_mode:
            raise RuntimeError(
                "set_rrt_pose() is forbidden while physics_mode is active"
            )
        self._rrt = np.array(pose, dtype=np.float64)
        self._rrt = np.clip(self._rrt, self._limits[:, 0], self._limits[:, 1])
        self._write_mujoco_state()

    def set_sst_pose(self, pose: tuple[float, float, float]) -> None:
        """Update SST agent pose ``(x, y, heading)``."""
        if self._physics_mode:
            raise RuntimeError(
                "set_sst_pose() is forbidden while physics_mode is active"
            )
        self._sst = np.array(pose, dtype=np.float64)
        self._sst = np.clip(self._sst, self._limits[:, 0], self._limits[:, 1])
        self._write_mujoco_state()

    def get_rrt_pose(self) -> npt.NDArray[np.float64]:
        """Return RRT* pose copy."""
        return self._rrt.copy()

    def get_sst_pose(self) -> npt.NDArray[np.float64]:
        """Return SST pose copy."""
        return self._sst.copy()

    def set_dummy_pose(self, pose: tuple[float, float, float]) -> None:
        """Update straight-line dummy pose ``(x, y, heading)``."""
        if self._physics_mode:
            raise RuntimeError(
                "set_dummy_pose() is forbidden while physics_mode is active"
            )
        self._dummy = np.array(pose, dtype=np.float64)
        self._dummy = np.clip(
            self._dummy, self._limits[:, 0], self._limits[:, 1]
        )
        self._write_mujoco_state()

    def get_dummy_pose(self) -> npt.NDArray[np.float64]:
        """Return dummy pose copy."""
        return self._dummy.copy()

    def get_joint_velocities(self) -> npt.NDArray[np.float64]:
        """Return the most recent nine-vector world-frame command."""
        return self._velocities.copy()

    def get_collision_forces_n(self) -> tuple[float, float, float]:
        """Return ``(rrt, sst, dummy)`` peak obstacle-contact force [N].

        Computed every ``step_physics()`` tick from real MuJoCo contacts
        (independent of whether JSONL contact logging is enabled) so a
        collision *monitor* can stop an agent's control loop after an
        actual impact rather than pre-emptively blocking its motion.
        """
        return (
            self._collision_forces_n.get(_RRT_COLLISION_GEOM, 0.0),
            self._collision_forces_n.get(_SST_COLLISION_GEOM, 0.0),
            self._collision_forces_n.get(_DUMMY_COLLISION_GEOM, 0.0),
        )

    def step_physics(
        self,
        velocities: npt.NDArray[np.float64],
        *,
        substeps: int | None = None,
    ) -> npt.NDArray[np.float64]:
        """Advance agents from a length-9 world-frame command ``(vx,vy,ω)×3``.

        Converts each agent's world velocity to body ``(v, ω)``, maps to
        wheel rates, writes the six wheel actuators, then runs pure
        ``mj_step`` substeps.
        """
        if not self._physics_mode:
            raise RuntimeError("step_physics() requires physics_mode=True")
        if self._model is None or self._data is None or self._mujoco is None:
            raise RuntimeError("step_physics() requires the mujoco package")

        v = np.asarray(velocities, dtype=np.float64).reshape(9)
        if len(self._actuator_ids) != 6:
            raise RuntimeError(
                "Dubins race MJCF must expose six wheel actuators"
            )

        self._velocities = v.copy()
        poses = (self._rrt, self._sst, self._dummy)
        wheel_ctrl = np.empty(6, dtype=np.float64)
        for agent_idx, pose in enumerate(poses):
            base = 3 * agent_idx
            yaw = float(pose[2])
            forward = _world_velocity_to_body_forward(
                float(v[base]),
                float(v[base + 1]),
                yaw,
            )
            omega = float(v[base + 2])
            omega_l, omega_r = body_velocity_to_wheel_rates(
                forward,
                omega,
                wheel_radius_m=_WHEEL_RADIUS_M,
                track_width_m=_TRACK_WIDTH_M,
            )
            omega_l, omega_r = clip_wheel_rates(
                omega_l,
                omega_r,
                limit_rad_s=_WHEEL_CTRL_LIMIT_RAD_S,
            )
            wheel_ctrl[2 * agent_idx] = omega_l
            wheel_ctrl[2 * agent_idx + 1] = omega_r

        for idx, act_id in enumerate(self._actuator_ids):
            self._data.ctrl[act_id] = float(wheel_ctrl[idx])

        step_count = (
            self._substeps_per_tick if substeps is None else int(substeps)
        )
        if step_count <= 0:
            raise ValueError("substeps must be positive")

        for _ in range(step_count):
            self._mujoco.mj_step(self._model, self._data)

        self._collision_forces_n = agent_obstacle_contact_forces(
            self._model,
            self._data,
            self._mujoco,
            _DUBINS_COLLISION_GEOMS,
        )

        if self._contact_logger is not None:
            self._contact_logger.record_tick(
                self._model,
                self._data,
                self._mujoco,
            )

        self._read_state_from_mujoco()
        return np.concatenate([self._rrt, self._sst, self._dummy]).astype(
            np.float64
        )

    def _load_mujoco_optional(self) -> None:
        try:
            import mujoco
        except ImportError:
            return

        try:
            self._mujoco = mujoco
            self._model = mujoco.MjModel.from_xml_path(
                str(self._mjcf_path.resolve())
            )
            self._data = mujoco.MjData(self._model)
            qpos_adrs, _qvel_adrs = _bind_joint_addresses(
                self._model,
                mujoco,
                list(_DUBINS_BASE_JOINTS),
            )
            for name, adr in zip(_DUBINS_BASE_JOINTS, qpos_adrs, strict=True):
                self._qpos_adrs[name] = adr
        except Exception:
            self._mujoco = None
            self._model = None
            self._data = None
            self._qpos_adrs = {}

    def _write_freejoint_pose(
        self,
        joint_name: str,
        pose: npt.NDArray[np.float64],
    ) -> None:
        """Write planar ``(x, y, yaw)`` into a freejoint qpos block."""
        if self._data is None:
            return
        adr = self._qpos_adrs.get(joint_name)
        if adr is None:
            return
        qw, qx, qy, qz = _yaw_to_quat_wxyz(float(pose[2]))
        self._data.qpos[adr] = float(pose[0])
        self._data.qpos[adr + 1] = float(pose[1])
        self._data.qpos[adr + 2] = _AGENT_BASE_Z_M
        self._data.qpos[adr + 3] = qw
        self._data.qpos[adr + 4] = qx
        self._data.qpos[adr + 5] = qy
        self._data.qpos[adr + 6] = qz

    def _read_freejoint_pose(self, joint_name: str) -> npt.NDArray[np.float64]:
        """Read planar ``(x, y, yaw)`` from a freejoint qpos block."""
        if self._data is None:
            return np.zeros(3, dtype=np.float64)
        adr = self._qpos_adrs[joint_name]
        x = float(self._data.qpos[adr])
        y = float(self._data.qpos[adr + 1])
        qw, qx, qy, qz = (float(v) for v in self._data.qpos[adr + 3 : adr + 7])
        yaw = _quat_wxyz_to_yaw(qw, qx, qy, qz)
        return np.array([x, y, yaw], dtype=np.float64)

    def _seed_mujoco_state(self) -> None:
        """Write initial agent poses once at construction."""
        if self._model is None or self._data is None or self._mujoco is None:
            return
        self._write_freejoint_pose(_RRT_BASE_JOINT, self._rrt)
        self._write_freejoint_pose(_SST_BASE_JOINT, self._sst)
        self._write_freejoint_pose(_DUMMY_BASE_JOINT, self._dummy)
        self._mujoco.mj_forward(self._model, self._data)

    def _read_state_from_mujoco(self) -> None:
        if self._data is None or not self._qpos_adrs:
            return
        # Freejoint agents: keep raw XY (workspace clip desyncs control from
        # physics). Only wrap yaw into ``[-pi, pi]``.
        self._rrt = self._clamp_freejoint_pose(
            self._read_freejoint_pose(_RRT_BASE_JOINT)
        )
        self._sst = self._clamp_freejoint_pose(
            self._read_freejoint_pose(_SST_BASE_JOINT)
        )
        self._dummy = self._clamp_freejoint_pose(
            self._read_freejoint_pose(_DUMMY_BASE_JOINT)
        )

    @staticmethod
    def _clamp_freejoint_pose(
        pose: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Return ``(x, y, wrap(yaw))`` without inventing workspace walls."""
        yaw = float(pose[2])
        return np.array(
            [
                float(pose[0]),
                float(pose[1]),
                math.atan2(math.sin(yaw), math.cos(yaw)),
            ],
            dtype=np.float64,
        )

    def _write_mujoco_state(self) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            return
        if self._physics_mode:
            raise RuntimeError(
                "pose injection is forbidden while physics_mode is active"
            )
        self._write_freejoint_pose(_RRT_BASE_JOINT, self._rrt)
        self._write_freejoint_pose(_SST_BASE_JOINT, self._sst)
        self._write_freejoint_pose(_DUMMY_BASE_JOINT, self._dummy)
        self._mujoco.mj_forward(self._model, self._data)


def make_dubins_race_bridge_core(
    *,
    mjcf_path: str | pathlib.Path | None = None,
    initial_rrt: npt.NDArray[np.float64] | None = None,
    initial_sst: npt.NDArray[np.float64] | None = None,
    initial_dummy: npt.NDArray[np.float64] | None = None,
    physics_config: PhysicsBridgeConfig | None = None,
) -> DubinsRaceBridgeCore:
    """Build a triple-agent MuJoCo bridge for SC-v11."""
    return DubinsRaceBridgeCore(
        mjcf_path=mjcf_path,
        initial_rrt=initial_rrt,
        initial_sst=initial_sst,
        initial_dummy=initial_dummy,
        physics_config=physics_config,
    )


def _resolve_config_path(config_path: str | None) -> str:
    if config_path is not None:
        return config_path
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("fret")
        return os.path.join(share, "config", "simulation", "mujoco.yml")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "..", "config", "simulation", "mujoco.yml")


def _load_bridge_config(config_path: str) -> dict[str, Any]:
    import yaml

    with open(config_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        return {}
    for section in data.values():
        if isinstance(section, dict):
            params = section.get("ros__parameters", {})
            if isinstance(params, dict):
                return params
    return {}


def _load_merged_bridge_config(config_path: str) -> dict[str, Any]:
    """Load bridge YAML plus optional ``mujoco_physics.yml`` companion tables."""
    cfg = dict(_load_bridge_config(config_path))
    physics_path = pathlib.Path(config_path).with_name("mujoco_physics.yml")
    if physics_path.is_file():
        cfg.update(_load_bridge_config(str(physics_path)))
    return cfg


class MuJoCoBridgeNode:
    """ROS 2 node bridging MuJoCo joint I/O to the FRET graph.

    Subscribes:
        /joint_commands  (``std_msgs/Float64MultiArray``, BEST_EFFORT)

    Publishes:
        /joint_states    (``sensor_msgs/JointState``, default 50 Hz)

    Args:
        config_path: Optional path to ``mujoco.yml``.
        model: Robot model override.
        scenario: Scenario stem override.
        mjcf_path: Optional MJCF path override.
    """

    def __init__(
        self,
        config_path: str | None = None,
        *,
        model: str | None = None,
        scenario: str | None = None,
        mjcf_path: str | None = None,
    ) -> None:
        import rclpy
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._rclpy = rclpy

        class _Node(rclpy.node.Node):  # type: ignore[misc]
            pass

        self._node: rclpy.node.Node = _Node("mujoco_bridge_node")

        resolved = _resolve_config_path(config_path)
        cfg = _load_merged_bridge_config(resolved)

        self._node.declare_parameter(
            "model", model or str(cfg.get("model", "dubins"))
        )
        self._node.declare_parameter(
            "scenario", scenario or str(cfg.get("scenario", "dubins_race"))
        )
        self._node.declare_parameter(
            "update_rate",
            float(cfg.get("update_rate", _DEFAULT_UPDATE_RATE_HZ)),
        )
        self._node.declare_parameter(
            "initial_joint_positions",
            list(cfg.get("initial_joint_positions", [0.0, 0.0, 0.0])),
        )
        self._node.declare_parameter(
            "physics_mode",
            _parse_bool(cfg.get("physics_mode", False)),
        )
        self._node.declare_parameter(
            "substeps_per_tick",
            int(cfg.get("substeps_per_tick", _DEFAULT_SUBSTEPS_PER_TICK)),
        )

        model_name = str(
            self._node.get_parameter("model")
            .get_parameter_value()
            .string_value
        )
        scenario_name = str(
            self._node.get_parameter("scenario")
            .get_parameter_value()
            .string_value
        )
        self._update_rate = float(
            self._node.get_parameter("update_rate")
            .get_parameter_value()
            .double_value
        )
        initial = (
            self._node.get_parameter("initial_joint_positions")
            .get_parameter_value()
            .double_array_value
        )
        q0 = np.asarray(initial, dtype=np.float64)

        physics_mode = _parse_bool(
            self._node.get_parameter("physics_mode").value
        )
        cfg_for_physics = dict(cfg)
        cfg_for_physics["physics_mode"] = physics_mode
        cfg_for_physics["substeps_per_tick"] = int(
            self._node.get_parameter("substeps_per_tick").value
        )
        physics_config = physics_config_from_bridge_yaml(
            cfg_for_physics,
            model_name,
            physics_mode=physics_mode,
        )

        self._core = make_mujoco_bridge_core(
            model_name,
            scenario_name,
            mjcf_path=mjcf_path,
            initial_positions=q0,
        )
        self._latest_cmd = np.zeros(
            self._core.get_positions().shape, dtype=np.float64
        )
        if physics_config.physics_mode:
            self._core.configure_physics(physics_config)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._node.create_subscription(
            Float64MultiArray,
            "/joint_commands",
            self._command_callback,
            cmd_qos,
        )

        self._state_pub = self._node.create_publisher(
            JointState, "/joint_states", 10
        )
        self._timer = self._node.create_timer(
            1.0 / self._update_rate,
            self._timer_callback,
        )
        self._node.get_logger().info(
            "MuJoCoBridgeNode ready: "
            f"model={model_name}, scenario={scenario_name}, "
            f"mjcf={self._core.mjcf_path.name}, "
            f"rate={self._update_rate} Hz, "
            f"physics_mode={self._core.physics_mode}, "
            f"mujoco_runtime={self._core.has_mujoco_runtime}"
        )

    def _command_callback(self, msg: Any) -> None:
        data = list(getattr(msg, "data", []))
        dof = len(self._latest_cmd)
        if len(data) < dof:
            self._node.get_logger().warning(
                f"Received /joint_commands with {len(data)} values; expected {dof}."
            )
            return
        self._latest_cmd = np.asarray(data[:dof], dtype=np.float64)

    def _timer_callback(self) -> None:
        from sensor_msgs.msg import JointState

        dt = 1.0 / self._update_rate
        positions = self._core.step(self._latest_cmd, dt)
        velocities = self._core.get_velocities()

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.name = self._core.joint_names
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        self._state_pub.publish(msg)

    def spin(self) -> None:
        """Spin the underlying ROS 2 node until shutdown."""
        try:
            self._rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``mujoco_bridge`` executable."""
    import rclpy

    rclpy.init(args=args)
    node = MuJoCoBridgeNode()
    node.spin()


__all__ = [
    "ActuatorTable",
    "ContactLogConfig",
    "DubinsRaceBridgeCore",
    "MuJoCoBridgeCore",
    "MuJoCoBridgeNode",
    "PhysicsBridgeConfig",
    "PhysicsContactLogger",
    "contact_log_config_from_bridge_yaml",
    "integrate_joint_velocities",
    "make_dubins_race_bridge_core",
    "make_mujoco_bridge_core",
    "physics_config_from_bridge_yaml",
    "resolve_mjcf_path",
]
