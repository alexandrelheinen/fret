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

import os
import pathlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from fret.control.kinematics_ppp import PPPKinematics
from fret.ros.mujoco_physics_log import (
    ContactLogConfig,
    PhysicsContactLogger,
    contact_log_config_from_bridge_yaml,
)

if TYPE_CHECKING:
    from fret.control.grasp_magnet import MagneticGraspFSM

# MJCF preview scale (1:5) limits for ppp_warehouse.xml.
_PPP_MJCF_LIMITS: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 12.0],
        [0.0, 5.0],
        [0.0, 3.0],
    ],
    dtype=np.float64,
)

# Dubins race workspace limits for dubins_race.xml.
_DUBINS_RACE_LIMITS: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 80.0],
        [0.0, 80.0],
        [-np.pi, np.pi],
    ],
    dtype=np.float64,
)

_RRT_JOINT_NAMES: list[str] = [
    "rrt_joint_x",
    "rrt_joint_y",
    "rrt_joint_yaw",
]
_SST_JOINT_NAMES: list[str] = [
    "sst_joint_x",
    "sst_joint_y",
    "sst_joint_yaw",
]

_DEFAULT_UPDATE_RATE_HZ: float = 50.0
_DEFAULT_MJCF_TIMESTEP_S: float = 0.002
_DEFAULT_SUBSTEPS_PER_TICK: int = 25


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


@dataclass(frozen=True)
class CargoWeldConfig:
    """PPP cargo equality-weld settings (v1.2, T12-04)."""

    equality_name: str
    body_parent: str
    body_child: str


def cargo_weld_config_from_bridge_yaml(cfg: dict[str, Any]) -> CargoWeldConfig:
    """Build :class:`CargoWeldConfig` from merged bridge YAML."""
    section = cfg.get("cargo_weld")
    if not isinstance(section, dict):
        raise ValueError("cargo_weld section missing from mujoco_physics.yml")
    equality_name = str(section.get("equality_name", "")).strip()
    body_parent = str(section.get("body_parent", "")).strip()
    body_child = str(section.get("body_child", "")).strip()
    if not equality_name or not body_parent or not body_child:
        raise ValueError(
            "cargo_weld requires equality_name, body_parent, and body_child"
        )
    return CargoWeldConfig(
        equality_name=equality_name,
        body_parent=body_parent,
        body_child=body_child,
    )


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
        model: Robot model name (e.g. ``ppp``).
        scenario: Scenario stem (e.g. ``ppp_warehouse``).
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
        model: Robot model name (``ppp`` supported in v1.0).
        scenario: Scenario stem used for MJCF resolution.
        mjcf_path: Optional explicit MJCF path override.
        initial_positions: Optional initial joint configuration.
        physics_config: Optional physics SITL settings (v1.2).

    Returns:
        Configured ``MuJoCoBridgeCore`` instance.

    Raises:
        ValueError: If ``model`` is not recognised.
    """
    if model == "ppp":
        kin = PPPKinematics()
        resolved = resolve_mjcf_path(model, scenario, mjcf_path)
        q0 = (
            np.zeros(kin.dof, dtype=np.float64)
            if initial_positions is None
            else np.asarray(initial_positions, dtype=np.float64)
        )
        if q0.shape != (kin.dof,):
            raise ValueError(f"initial_positions must have shape ({kin.dof},)")

        core = MuJoCoBridgeCore(
            mjcf_path=resolved,
            joint_names=kin.joint_names,
            limits=_PPP_MJCF_LIMITS,
            initial_positions=q0,
        )
        if physics_config is not None:
            core.configure_physics(physics_config)
        return core

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
        self._cargo_eq_id: int | None = None
        self._cargo_qpos_adr: int | None = None
        self._cargo_geom_id: int | None = None
        self._cargo_geom_contype: int = 1
        self._cargo_geom_conaffinity: int = 1
        self._cargo_weld_active = False
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

    def configure_cargo_weld(self, config: CargoWeldConfig) -> None:
        """Bind the PPP cargo equality weld for grasp physics (T12-04)."""
        if self._model is None or self._mujoco is None or self._data is None:
            raise RuntimeError(
                "configure_cargo_weld requires the optional mujoco package"
            )
        eq_id = self._mujoco.mj_name2id(
            self._model,
            self._mujoco.mjtObj.mjOBJ_EQUALITY,
            config.equality_name,
        )
        if eq_id < 0:
            raise ValueError(
                f"Equality constraint not found in MJCF: {config.equality_name}"
            )
        joint_id = self._mujoco.mj_name2id(
            self._model,
            self._mujoco.mjtObj.mjOBJ_JOINT,
            "cargo_free",
        )
        if joint_id < 0:
            raise ValueError("Cargo freejoint not found in MJCF: cargo_free")
        geom_id = self._mujoco.mj_name2id(
            self._model,
            self._mujoco.mjtObj.mjOBJ_GEOM,
            "cargo_box",
        )
        if geom_id < 0:
            raise ValueError("Cargo geom not found in MJCF: cargo_box")
        self._cargo_eq_id = int(eq_id)
        self._cargo_qpos_adr = int(self._model.jnt_qposadr[joint_id])
        self._cargo_geom_id = int(geom_id)
        self._cargo_geom_contype = int(self._model.geom_contype[geom_id])
        self._cargo_geom_conaffinity = int(
            self._model.geom_conaffinity[geom_id]
        )
        self._cargo_weld_active = bool(self._data.eq_active[eq_id])

    def set_cargo_weld_active(self, active: bool) -> None:
        """Enable or disable the cargo weld equality constraint."""
        if (
            self._cargo_eq_id is None
            or self._model is None
            or self._data is None
        ):
            return
        if self._mujoco is None:
            return
        flag = bool(active)
        if flag == self._cargo_weld_active:
            return
        self._data.eq_active[self._cargo_eq_id] = int(flag)
        self._cargo_weld_active = flag
        self._mujoco.mj_forward(self._model, self._data)

    def set_cargo_contacts_enabled(self, enabled: bool) -> None:
        """Enable or disable ``cargo_box`` collision participation."""
        if self._cargo_geom_id is None or self._model is None:
            return
        if enabled:
            self._model.geom_contype[self._cargo_geom_id] = (
                self._cargo_geom_contype
            )
            self._model.geom_conaffinity[self._cargo_geom_id] = (
                self._cargo_geom_conaffinity
            )
        else:
            self._model.geom_contype[self._cargo_geom_id] = 0
            self._model.geom_conaffinity[self._cargo_geom_id] = 0
        if self._mujoco is not None and self._data is not None:
            self._mujoco.mj_forward(self._model, self._data)

    def seed_cargo_pose(self, position: npt.NDArray[np.float64]) -> None:
        """Set initial cargo freejoint pose before simulation starts."""
        if self._physics_mode:
            raise RuntimeError(
                "seed_cargo_pose() is forbidden while physics_mode is active"
            )
        self._write_cargo_pose(position)

    def set_cargo_pose(self, position: npt.NDArray[np.float64]) -> None:
        """Write cargo freejoint pose in kinematic mode (v1.0 mirror path)."""
        if self._physics_mode:
            raise RuntimeError(
                "set_cargo_pose() is forbidden while physics_mode is active"
            )
        self._write_cargo_pose(position)

    def sync_cargo_grasp(
        self,
        *,
        is_welded: bool,
        cargo_pose: npt.NDArray[np.float64],
        cargo_in_transport: bool = False,
    ) -> None:
        """Apply grasp FSM output to cargo weld or kinematic pose (T12-04)."""
        if self._physics_mode:
            self.set_cargo_weld_active(is_welded)
            self.set_cargo_contacts_enabled(
                not (cargo_in_transport or is_welded)
            )
            if cargo_in_transport and not is_welded:
                self._write_cargo_pose(cargo_pose)
            return
        self.set_cargo_weld_active(False)
        self.set_cargo_pose(cargo_pose)

    def _write_cargo_pose(self, position: npt.NDArray[np.float64]) -> None:
        if (
            self._cargo_qpos_adr is None
            or self._model is None
            or self._data is None
        ):
            return
        if self._mujoco is None:
            return
        pos = np.asarray(position, dtype=np.float64).reshape(3)
        adr = self._cargo_qpos_adr
        self._data.qpos[adr : adr + 3] = pos
        self._data.qpos[adr + 3 : adr + 7] = np.array(
            [1.0, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )
        self._mujoco.mj_forward(self._model, self._data)

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
        self._model = mujoco.MjModel.from_xml_path(str(self._mjcf_path))
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
    """MuJoCo state mirror for the dual-agent Dubins race scene.

    In kinematic mode, writes RRT* and SST vehicle poses into
    ``dubins_race.xml`` joint coordinates without integrating dynamics.

    In physics mode (v1.2), six velocity actuators are driven via
    :meth:`step_physics` and poses are read from simulated ``qpos``.
    """

    _JOINT_NAMES: tuple[str, ...] = tuple(_RRT_JOINT_NAMES + _SST_JOINT_NAMES)

    def __init__(
        self,
        *,
        mjcf_path: str | pathlib.Path | None = None,
        initial_rrt: npt.NDArray[np.float64] | None = None,
        initial_sst: npt.NDArray[np.float64] | None = None,
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
        self._mujoco: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._joint_adrs: dict[str, int] = {}
        self._qvel_adrs: dict[str, int] = {}
        self._physics_mode = False
        self._substeps_per_tick = _DEFAULT_SUBSTEPS_PER_TICK
        self._actuator_ids: list[int] = []
        self._velocities = np.zeros(6, dtype=np.float64)
        self._contact_logger: PhysicsContactLogger | None = None
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

    def get_joint_velocities(self) -> npt.NDArray[np.float64]:
        """Return the most recent six-DOF velocity commands."""
        return self._velocities.copy()

    def step_physics(
        self,
        velocities: npt.NDArray[np.float64],
        *,
        substeps: int | None = None,
    ) -> npt.NDArray[np.float64]:
        """Advance both agents with six velocity actuator commands."""
        if not self._physics_mode:
            raise RuntimeError("step_physics() requires physics_mode=True")
        if self._model is None or self._data is None or self._mujoco is None:
            raise RuntimeError("step_physics() requires the mujoco package")

        v = np.asarray(velocities, dtype=np.float64).reshape(6)
        if len(self._actuator_ids) != 6:
            raise RuntimeError("Dubins race MJCF must expose six actuators")

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
        return np.concatenate([self._rrt, self._sst]).astype(np.float64)

    def _load_mujoco_optional(self) -> None:
        try:
            import mujoco
        except ImportError:
            return

        try:
            self._mujoco = mujoco
            self._model = mujoco.MjModel.from_xml_path(str(self._mjcf_path))
            self._data = mujoco.MjData(self._model)
            qpos_adrs, qvel_adrs = _bind_joint_addresses(
                self._model,
                mujoco,
                list(self._JOINT_NAMES),
            )
            for name, adr in zip(self._JOINT_NAMES, qpos_adrs, strict=True):
                self._joint_adrs[name] = adr
            for name, adr in zip(self._JOINT_NAMES, qvel_adrs, strict=True):
                self._qvel_adrs[name] = adr
        except Exception:
            self._mujoco = None
            self._model = None
            self._data = None
            self._joint_adrs = {}
            self._qvel_adrs = {}

    def _seed_mujoco_state(self) -> None:
        """Write initial agent poses once at construction."""
        if self._model is None or self._data is None or self._mujoco is None:
            return
        mapping = {
            "rrt_joint_x": float(self._rrt[0]),
            "rrt_joint_y": float(self._rrt[1]),
            "rrt_joint_yaw": float(self._rrt[2]),
            "sst_joint_x": float(self._sst[0]),
            "sst_joint_y": float(self._sst[1]),
            "sst_joint_yaw": float(self._sst[2]),
        }
        for name, value in mapping.items():
            adr = self._joint_adrs.get(name)
            if adr is not None:
                self._data.qpos[adr] = value
        self._mujoco.mj_forward(self._model, self._data)

    def _read_state_from_mujoco(self) -> None:
        if self._data is None:
            return
        rrt = np.empty(3, dtype=np.float64)
        sst = np.empty(3, dtype=np.float64)
        for idx, name in enumerate(_RRT_JOINT_NAMES):
            rrt[idx] = float(self._data.qpos[self._joint_adrs[name]])
        for idx, name in enumerate(_SST_JOINT_NAMES):
            sst[idx] = float(self._data.qpos[self._joint_adrs[name]])
        self._rrt = np.clip(rrt, self._limits[:, 0], self._limits[:, 1])
        self._sst = np.clip(sst, self._limits[:, 0], self._limits[:, 1])

    def _write_mujoco_state(self) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            return
        if self._physics_mode:
            raise RuntimeError(
                "pose injection is forbidden while physics_mode is active"
            )
        mapping = {
            "rrt_joint_x": float(self._rrt[0]),
            "rrt_joint_y": float(self._rrt[1]),
            "rrt_joint_yaw": float(self._rrt[2]),
            "sst_joint_x": float(self._sst[0]),
            "sst_joint_y": float(self._sst[1]),
            "sst_joint_yaw": float(self._sst[2]),
        }
        for name, value in mapping.items():
            adr = self._joint_adrs.get(name)
            if adr is not None:
                self._data.qpos[adr] = value
        self._mujoco.mj_forward(self._model, self._data)


def make_dubins_race_bridge_core(
    *,
    mjcf_path: str | pathlib.Path | None = None,
    initial_rrt: npt.NDArray[np.float64] | None = None,
    initial_sst: npt.NDArray[np.float64] | None = None,
    physics_config: PhysicsBridgeConfig | None = None,
) -> DubinsRaceBridgeCore:
    """Build a dual-agent MuJoCo bridge for SC-v11."""
    return DubinsRaceBridgeCore(
        mjcf_path=mjcf_path,
        initial_rrt=initial_rrt,
        initial_sst=initial_sst,
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
            "model", model or str(cfg.get("model", "ppp"))
        )
        self._node.declare_parameter(
            "scenario", scenario or str(cfg.get("scenario", "ppp_warehouse"))
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
        self._ppp_grasp: MagneticGraspFSM | None = None
        self._ppp_kin: PPPKinematics | None = None
        self._ppp_box_anchor: npt.NDArray[np.float64] | None = None
        self._ppp_goal: npt.NDArray[np.float64] | None = None
        self._ppp_grasp_captured = False
        if model_name == "ppp":
            self._init_ppp_grasp(scenario_name, q0, cfg)
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

    def _init_ppp_grasp(
        self,
        scenario_name: str,
        q0: npt.NDArray[np.float64],
        cfg: dict[str, Any],
    ) -> None:
        """Load PPP grasp FSM and seed cargo pose for SITL (T12-04)."""
        from fret.config_loader import load_scenario_bundle
        from fret.control.grasp_magnet import (
            MagneticGraspFSM,
            parse_grasp_config,
        )
        from fret.control.kinematics_ppp import PPPKinematics
        from fret.sitl_config import scenario_config_path

        bundle = load_scenario_bundle(scenario_config_path(scenario_name))
        if bundle.grasp is None:
            return

        grasp_cfg = parse_grasp_config(bundle.grasp)
        params = bundle.parameters
        start = np.asarray(
            params.get("start_configuration", q0),
            dtype=np.float64,
        )
        goal = np.asarray(params["goal_configuration"], dtype=np.float64)
        self._ppp_kin = PPPKinematics()
        self._ppp_grasp = MagneticGraspFSM(grasp_cfg)
        self._ppp_box_anchor = np.array(
            [
                start[0],
                start[1],
                float(grasp_cfg.box_half_extent[2]),
            ],
            dtype=np.float64,
        )
        self._ppp_goal = goal
        if self._core.has_mujoco_runtime and "cargo_weld" in cfg:
            self._core.configure_cargo_weld(
                cargo_weld_config_from_bridge_yaml(cfg)
            )
            self._core.seed_cargo_pose(self._ppp_box_anchor)
        self._ppp_grasp.begin_transport()

    def _sync_ppp_cargo_grasp(self, q: npt.NDArray[np.float64]) -> None:
        """Advance grasp FSM and mirror cargo weld / pose in MuJoCo."""
        if (
            self._ppp_grasp is None
            or self._ppp_kin is None
            or self._ppp_box_anchor is None
            or self._ppp_goal is None
        ):
            return

        from fret.control.grasp_magnet import GraspState

        ee = self._ppp_kin.forward_kinematics(q)[:3, 3]
        state = self._ppp_grasp.update(
            ee, self._ppp_box_anchor, self._ppp_goal
        )
        if state == GraspState.TRANSPORT:
            self._ppp_grasp_captured = True
        if self._ppp_grasp.is_welded or self._ppp_grasp_captured:
            cargo_pose = self._ppp_grasp.cargo_position
        else:
            cargo_pose = self._ppp_box_anchor
        self._core.sync_cargo_grasp(
            is_welded=self._ppp_grasp.is_welded,
            cargo_pose=cargo_pose,
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
        q_before = self._core.get_positions()
        self._sync_ppp_cargo_grasp(q_before)
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
    "CargoWeldConfig",
    "ContactLogConfig",
    "DubinsRaceBridgeCore",
    "MuJoCoBridgeCore",
    "MuJoCoBridgeNode",
    "PhysicsBridgeConfig",
    "PhysicsContactLogger",
    "cargo_weld_config_from_bridge_yaml",
    "contact_log_config_from_bridge_yaml",
    "integrate_joint_velocities",
    "make_dubins_race_bridge_core",
    "make_mujoco_bridge_core",
    "physics_config_from_bridge_yaml",
    "resolve_mjcf_path",
]
