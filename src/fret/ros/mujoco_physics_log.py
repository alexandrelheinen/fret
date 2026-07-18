"""MuJoCo physics contact logging and shutdown metrics (v1.2, T12-05).

Appends JSONL contact records after physics ticks and writes a summary
``metrics.json`` at scenario shutdown.  Satisfies FR-SIM-08.
"""

from __future__ import annotations

import json
import pathlib
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import numpy.typing as npt

_PENETRATION_TOLERANCE_M: float = 0.001
_OBSTACLE_GEOM_PREFIXES: tuple[str, ...] = ("obs_", "str_")
_AGENT_GEOM_NAMES: frozenset[str] = frozenset(
    {
        "cargo_box",
        "ee_gripper",
        "ee_spreader",
        "z_column",
        "rrt_collision",
        "sst_collision",
    }
)


@dataclass(frozen=True)
class ContactLogConfig:
    """Runtime contact logging settings loaded from ``mujoco.yml``."""

    enabled: bool
    log_path: pathlib.Path
    metrics_path: pathlib.Path
    scenario_id: str
    physics_mode: bool = True


@dataclass
class PhysicsMetrics:
    """Aggregated physics SITL metrics written at shutdown."""

    scenario_id: str
    physics_mode: bool
    sim_time_final: float = 0.0
    wall_time_elapsed: float = 0.0
    max_tracking_error_m: float | None = None
    contact_event_count: int = 0
    max_contact_force_n: float = 0.0
    penetration_violations: int = 0

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable metrics mapping."""
        payload: dict[str, Any] = {
            "scenario_id": self.scenario_id,
            "physics_mode": self.physics_mode,
            "sim_time_final": float(self.sim_time_final),
            "wall_time_elapsed": float(self.wall_time_elapsed),
            "contact_event_count": int(self.contact_event_count),
            "max_contact_force_n": float(self.max_contact_force_n),
            "penetration_violations": int(self.penetration_violations),
        }
        if self.max_tracking_error_m is not None:
            payload["max_tracking_error_m"] = float(self.max_tracking_error_m)
        return payload


def default_physics_artifact_dir(scenario_id: str) -> pathlib.Path:
    """Return ``/tmp/fret_physics/<scenario_id>``."""
    return pathlib.Path("/tmp/fret_physics") / scenario_id


def resolve_contact_log_path(
    cfg: dict[str, Any],
    scenario_id: str,
) -> pathlib.Path:
    """Resolve the JSONL contact log path from bridge YAML."""
    custom = str(cfg.get("contact_log_path", "")).strip()
    if custom:
        return pathlib.Path(custom)
    return default_physics_artifact_dir(scenario_id) / "contacts.jsonl"


def resolve_metrics_path(
    cfg: dict[str, Any],
    scenario_id: str,
) -> pathlib.Path:
    """Resolve the shutdown metrics JSON path from bridge YAML."""
    custom = str(cfg.get("metrics_path", "")).strip()
    if custom:
        return pathlib.Path(custom)
    return default_physics_artifact_dir(scenario_id) / "metrics.json"


def contact_log_config_from_bridge_yaml(
    cfg: dict[str, Any],
    scenario_id: str,
    *,
    physics_mode: bool = True,
    enabled: bool | None = None,
) -> ContactLogConfig:
    """Build :class:`ContactLogConfig` from merged bridge YAML."""
    if enabled is None:
        enabled = _parse_bool(cfg.get("contact_log_enabled", False))
    return ContactLogConfig(
        enabled=_parse_bool(enabled),
        log_path=resolve_contact_log_path(cfg, scenario_id),
        metrics_path=resolve_metrics_path(cfg, scenario_id),
        scenario_id=scenario_id,
        physics_mode=bool(physics_mode),
    )


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "on"}


def _geom_name(model: Any, mujoco: Any, geom_id: int) -> str:
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
    return str(name) if name is not None else f"geom_{geom_id}"


def _is_obstacle_geom(name: str) -> bool:
    return name.startswith(_OBSTACLE_GEOM_PREFIXES)


def _is_agent_geom(name: str) -> bool:
    return name in _AGENT_GEOM_NAMES


def _counts_as_penetration(geom1: str, geom2: str, dist: float) -> bool:
    """Return True for agent/obstacle interpenetration (excludes floor settle)."""
    if dist >= -_PENETRATION_TOLERANCE_M:
        return False
    g1_obs = _is_obstacle_geom(geom1)
    g2_obs = _is_obstacle_geom(geom2)
    g1_agent = _is_agent_geom(geom1)
    g2_agent = _is_agent_geom(geom2)
    return (g1_obs and g2_agent) or (g2_obs and g1_agent)


def _should_log_contact(geom1: str, geom2: str) -> bool:
    """Return True for obstacle or inter-agent contacts worth JSONL logging."""
    if _is_obstacle_geom(geom1) or _is_obstacle_geom(geom2):
        return True
    return _is_agent_geom(geom1) and _is_agent_geom(geom2)


def agent_obstacle_contact_forces(
    model: Any,
    data: Any,
    mujoco: Any,
    agent_geom_names: tuple[str, ...],
) -> dict[str, float]:
    """Return the peak obstacle-contact force [N] per agent geom this tick.

    Scans ``data.contact`` for real MuJoCo contacts between a named agent
    geom and any obstacle geom (``obs_`` / ``str_`` prefix), independent of
    whether JSONL contact logging is enabled. This backs a collision
    *monitor* that stops an agent's control loop after it actually hits
    something, rather than a pre-emptive occupancy-based motion block.
    """
    forces = dict.fromkeys(agent_geom_names, 0.0)
    if int(data.ncon) <= 0:
        return forces
    for idx in range(int(data.ncon)):
        contact = data.contact[idx]
        geom1 = _geom_name(model, mujoco, int(contact.geom1))
        geom2 = _geom_name(model, mujoco, int(contact.geom2))
        if geom1 in forces and _is_obstacle_geom(geom2):
            agent_name = geom1
        elif geom2 in forces and _is_obstacle_geom(geom1):
            agent_name = geom2
        else:
            continue
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, idx, force)
        force_norm = float(np.linalg.norm(force[:3]))
        if force_norm > forces[agent_name]:
            forces[agent_name] = force_norm
    return forces


class PhysicsContactLogger:
    """Append-only JSONL contact logger with rolling metrics."""

    def __init__(self, config: ContactLogConfig) -> None:
        self._config = config
        self.metrics = PhysicsMetrics(
            scenario_id=config.scenario_id,
            physics_mode=config.physics_mode,
        )
        self._wall_t0 = time.time()
        self._log_handle: Any | None = None
        self._penetration_prev_tick = False
        if config.enabled:
            config.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_handle = open(
                config.log_path,
                "a",
                encoding="utf-8",
            )

    @property
    def enabled(self) -> bool:
        """Return ``True`` when JSONL logging is active."""
        return self._config.enabled

    @property
    def log_path(self) -> pathlib.Path:
        """Resolved JSONL contact log path."""
        return self._config.log_path

    @property
    def metrics_path(self) -> pathlib.Path:
        """Resolved shutdown metrics JSON path."""
        return self._config.metrics_path

    def set_max_tracking_error_m(self, value: float) -> None:
        """Record the largest EE / cross-track error observed in the run."""
        current = self.metrics.max_tracking_error_m
        if current is None or value > current:
            self.metrics.max_tracking_error_m = float(value)

    def record_tick(
        self,
        model: Any,
        data: Any,
        mujoco: Any,
    ) -> None:
        """Scan ``data.contact`` after a physics control tick."""
        self.metrics.sim_time_final = float(data.time)
        if int(data.ncon) <= 0:
            self._penetration_prev_tick = False
            return

        tick_penetration = False
        for idx in range(int(data.ncon)):
            contact = data.contact[idx]
            geom1 = _geom_name(model, mujoco, int(contact.geom1))
            geom2 = _geom_name(model, mujoco, int(contact.geom2))
            dist = float(contact.dist)
            if _counts_as_penetration(geom1, geom2, dist):
                tick_penetration = True

            force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, idx, force)
            force_norm = float(np.linalg.norm(force[:3]))
            if force_norm <= 0.0:
                continue

            if _should_log_contact(geom1, geom2):
                self.metrics.contact_event_count += 1
                self.metrics.max_contact_force_n = max(
                    self.metrics.max_contact_force_n,
                    force_norm,
                )
            if self._log_handle is not None and _should_log_contact(
                geom1, geom2
            ):
                record = {
                    "sim_time": float(data.time),
                    "wall_time": time.time(),
                    "geom1": geom1,
                    "geom2": geom2,
                    "force_norm": force_norm,
                    "pos": [
                        float(contact.pos[0]),
                        float(contact.pos[1]),
                        float(contact.pos[2]),
                    ],
                }
                self._log_handle.write(json.dumps(record) + "\n")

        if tick_penetration:
            if self._penetration_prev_tick:
                self.metrics.penetration_violations += 1
            self._penetration_prev_tick = True
        else:
            self._penetration_prev_tick = False

    def close(
        self,
        *,
        max_tracking_error_m: float | None = None,
    ) -> pathlib.Path | None:
        """Flush logs and write ``metrics.json``."""
        if max_tracking_error_m is not None:
            self.set_max_tracking_error_m(max_tracking_error_m)

        self.metrics.wall_time_elapsed = time.time() - self._wall_t0
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None

        if not self._config.enabled and max_tracking_error_m is None:
            return None

        self._config.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._config.metrics_path, "w", encoding="utf-8") as fh:
            json.dump(self.metrics.to_dict(), fh, indent=2)
            fh.write("\n")
        return self._config.metrics_path

    def column_contacts_logged(self) -> bool:
        """Return True when a column collision produced non-zero force."""
        if not self._config.log_path.is_file():
            return False
        with open(self._config.log_path, encoding="utf-8") as fh:
            for line in fh:
                if not line.strip():
                    continue
                record = json.loads(line)
                geom1 = str(record.get("geom1", ""))
                geom2 = str(record.get("geom2", ""))
                if (
                    geom1.startswith("str_") or geom2.startswith("str_")
                ) and float(record.get("force_norm", 0.0)) > 0.0:
                    return True
        return False


__all__ = [
    "ContactLogConfig",
    "PhysicsContactLogger",
    "PhysicsMetrics",
    "agent_obstacle_contact_forces",
    "contact_log_config_from_bridge_yaml",
    "default_physics_artifact_dir",
    "resolve_contact_log_path",
    "resolve_metrics_path",
]
