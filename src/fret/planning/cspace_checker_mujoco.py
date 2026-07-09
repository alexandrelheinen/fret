"""MuJoCo collision checker for the PPP gantry (FR-PLN-02).

Uses MJCF geometry as the single source of truth for robot/obstacle
contacts.  Obstacle geoms are identified by name prefix (``obs_``,
``rack_`` collision boxes); robot geoms are all non-zero ``contype``
geoms outside that set.

Satisfies requirement FR-SIM-01 when ``collision_backend=mujoco``.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

from fret.control.grasp_magnet import GraspConfig
from fret.ros.mujoco_bridge import resolve_mjcf_path

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

_OBSTACLE_GEOM_PREFIXES: tuple[str, ...] = ("obs_", "rack_")
_CARGO_GEOM_NAMES: frozenset[str] = frozenset({"cargo_box"})
_STATIC_ENV_GEOM_NAMES: frozenset[str] = frozenset({"floor"})
_GEOM_DISTANCE_MAX_M: float = 20.0


@dataclass
class MujocoCheckerConfig:
    """Configuration for MuJoCo-backed PPP collision checks."""

    include_cargo: bool
    contact_radius: float
    grasp_config: GraspConfig | None = None
    mjcf_path: pathlib.Path | None = None
    scenario: str = "ppp_warehouse"
    workspace_bounds: (
        tuple[
            tuple[float, float],
            tuple[float, float],
            tuple[float, float],
        ]
        | None
    ) = None


class MujocoPPPCollisionChecker:
    """PPP checker that queries MuJoCo contacts at each configuration.

    Args:
        kinematics: ``Kinematics(model="ppp")`` instance.
        occupancy: Ignored — obstacles come from MJCF collision geoms.
        config: MuJoCo scene and cargo options.
    """

    def __init__(
        self,
        kinematics: Kinematics,
        occupancy: Any,
        config: MujocoCheckerConfig,
    ) -> None:
        del occupancy  # MJCF is authoritative for obstacles.
        self._kin = kinematics
        self._config = config
        self._dof = kinematics.dof
        self._mujoco, self._model, self._data = self._load_runtime()
        self._qpos_adrs = self._resolve_qpos_addresses()
        (
            self._base_robot_geom_ids,
            self._cargo_geom_ids,
            self._obstacle_geom_ids,
        ) = self._classify_geoms()

    @property
    def include_cargo(self) -> bool:
        """Whether welded cargo is included in collision checks."""
        return self._config.include_cargo

    def _load_runtime(self) -> tuple[Any, Any, Any]:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "MuJoCo collision backend requires the optional 'mujoco' "
                "package (pip install fret[sim])."
            ) from exc

        cfg = self._config
        mjcf = (
            pathlib.Path(cfg.mjcf_path)
            if cfg.mjcf_path is not None
            else resolve_mjcf_path("ppp", cfg.scenario, None)
        )
        model = mujoco.MjModel.from_xml_path(str(mjcf))
        data = mujoco.MjData(model)
        return mujoco, model, data

    def _resolve_qpos_addresses(self) -> list[int]:
        adrs: list[int] = []
        for name in self._kin.joint_names:
            jid = self._mujoco.mj_name2id(
                self._model,
                self._mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if jid < 0:
                raise ValueError(f"Joint not found in MJCF: {name}")
            adr = int(self._model.jnt_qposadr[jid])
            adrs.append(adr)
        return adrs

    def _classify_geoms(
        self,
    ) -> tuple[set[int], set[int], set[int]]:
        mujoco = self._mujoco
        model = self._model
        robot_ids: set[int] = set()
        cargo_ids: set[int] = set()
        obstacle_ids: set[int] = set()
        for gid in range(model.ngeom):
            if int(model.geom_contype[gid]) == 0:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if name is None:
                robot_ids.add(gid)
                continue
            if name.startswith(_OBSTACLE_GEOM_PREFIXES):
                obstacle_ids.add(gid)
            elif name in _CARGO_GEOM_NAMES:
                cargo_ids.add(gid)
            elif name in _STATIC_ENV_GEOM_NAMES or name == "goal_zone":
                continue
            else:
                robot_ids.add(gid)
        if not obstacle_ids:
            raise ValueError("MJCF contains no obstacle collision geoms")
        return robot_ids, cargo_ids, obstacle_ids

    def _active_robot_geom_ids(self) -> set[int]:
        """Robot geoms used for collision queries (cargo optional, FR-GSP-02)."""
        if self._config.include_cargo:
            return self._base_robot_geom_ids | self._cargo_geom_ids
        return set(self._base_robot_geom_ids)

    def _set_configuration(
        self, configuration: npt.NDArray[np.float64]
    ) -> None:
        for idx, adr in enumerate(self._qpos_adrs):
            self._data.qpos[adr] = float(configuration[idx])
        self._mujoco.mj_forward(self._model, self._data)

    def _has_robot_obstacle_contact(self) -> bool:
        mujoco = self._mujoco
        model = self._model
        data = self._data
        mujoco.mj_collision(model, data)
        robot_ids = self._active_robot_geom_ids()
        for i in range(int(data.ncon)):
            con = data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if (g1 in robot_ids and g2 in self._obstacle_geom_ids) or (
                g2 in robot_ids and g1 in self._obstacle_geom_ids
            ):
                return True
        return False

    def _min_robot_obstacle_separation(self) -> float:
        """Return minimum centre-to-surface separation over robot/obstacle geoms."""
        mujoco = self._mujoco
        fromto = np.zeros(6, dtype=np.float64)
        min_sep = float("inf")
        robot_ids = self._active_robot_geom_ids()
        for robot_id in robot_ids:
            for obstacle_id in self._obstacle_geom_ids:
                dist = mujoco.mj_geomDistance(
                    self._model,
                    self._data,
                    robot_id,
                    obstacle_id,
                    _GEOM_DISTANCE_MAX_M,
                    fromto,
                )
                if dist >= 0.0:
                    min_sep = min(min_sep, float(dist))
        if min_sep == float("inf"):
            return self._config.contact_radius
        return min_sep - self._config.contact_radius

    def is_collision_free(
        self, configuration: npt.NDArray[np.float64]
    ) -> bool:
        """Return True when MuJoCo reports no robot/obstacle contacts."""
        return self.clearance(configuration) > 0.0

    def clearance(self, configuration: npt.NDArray[np.float64]) -> float:
        """Return a signed clearance proxy from MuJoCo contact detection."""
        if configuration.shape != (self._dof,):
            raise ValueError(
                f"Expected shape ({self._dof},), got {configuration.shape}"
            )
        limits = self._kin.joint_limits
        if not (
            np.all(configuration >= limits[:, 0] - 1e-9)
            and np.all(configuration <= limits[:, 1] + 1e-9)
        ):
            return -1.0
        bounds = self._config.workspace_bounds
        if bounds is not None:
            (x_lo, x_hi), (y_lo, y_hi), (z_lo, z_hi) = bounds
            x, y, z = (
                float(configuration[0]),
                float(configuration[1]),
                float(configuration[2]),
            )
            if not (
                x_lo - 1e-9 <= x <= x_hi + 1e-9
                and y_lo - 1e-9 <= y <= y_hi + 1e-9
                and z_lo - 1e-9 <= z <= z_hi + 1e-9
            ):
                return -1.0
        self._set_configuration(configuration)
        if self._has_robot_obstacle_contact():
            return -self._config.contact_radius
        return self._min_robot_obstacle_separation()
