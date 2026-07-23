"""Telemetry session: agent registration + PlotJuggler CSV writer."""

from __future__ import annotations

import csv
import json
import os
import pathlib
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Mapping, Sequence

from fret.telemetry.naming import (
    SeriesId,
    build_series_id,
    parse_series_id,
    validate_agent_name,
    validate_series_id,
)

_SCHEMA_VERSION = 1
_FORMAT = "fret.telemetry.csv"


def default_telemetry_dir(run_id: str) -> pathlib.Path:
    """Return ``/tmp/fret_telemetry/<run_id>`` (or ``FRET_TELEMETRY_DIR``)."""
    root = os.environ.get("FRET_TELEMETRY_DIR", "").strip()
    base = pathlib.Path(root) if root else pathlib.Path("/tmp/fret_telemetry")
    return base / run_id


def make_run_id(scenario_id: str) -> str:
    """Build a unique run id: ``{scenario}_{UTC}_{pid}``."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return f"{scenario_id}_{stamp}_{os.getpid()}"


def telemetry_enabled_from_env() -> bool:
    """Return True when ``FRET_TELEMETRY_ENABLED`` is a truthy string."""
    raw = os.environ.get("FRET_TELEMETRY_ENABLED", "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


@dataclass
class _SeriesMeta:
    series_id: str
    unit: str
    quantity: str
    frame: str
    component: str
    agent: str
    derived: bool = False


@dataclass
class TelemetryAgent:
    """Named producer that registers the fields it will emit.

    Example::

        sst = session.register_agent("tb3_sst")
        sst.register("position", frame="enu", components=("x", "y"), unit="m")
        sst.register("orientation", frame="enu", components=("yaw",), unit="rad")
    """

    name: str
    _session: TelemetrySession
    kind: str = "agent"
    metadata: dict[str, Any] = field(default_factory=dict)

    def register(
        self,
        quantity: str,
        *,
        frame: str,
        components: Sequence[str],
        unit: str,
        derived: bool = False,
    ) -> list[str]:
        """Register one quantity and return the created series ids."""
        if not components:
            raise ValueError("components must be non-empty")
        ids: list[str] = []
        for component in components:
            series_id = build_series_id(
                self.name, quantity, frame, str(component)
            )
            self._session.register_series(
                series_id, unit=unit, derived=derived
            )
            ids.append(series_id)
        return ids

    def series_ids(self) -> list[str]:
        """Return series ids registered by this agent (sorted)."""
        return sorted(
            sid
            for sid, meta in self._session._series.items()  # noqa: SLF001
            if meta.agent == self.name
        )


class TelemetrySession:
    """Buffer samples and write PlotJuggler-compatible CSV + manifest.

    When ``enabled=False``, all methods are no-ops (no file I/O).
    """

    def __init__(
        self,
        *,
        run_id: str,
        scenario_id: str,
        enabled: bool = False,
        output_dir: pathlib.Path | None = None,
        dt_nominal_s: float | None = None,
        strict: bool = True,
        csv_basename: str = "telemetry",
    ) -> None:
        self.run_id = run_id
        self.scenario_id = scenario_id
        self.enabled = bool(enabled)
        self.strict = bool(strict)
        self.dt_nominal_s = dt_nominal_s
        self.csv_basename = csv_basename
        self.output_dir = (
            pathlib.Path(output_dir)
            if output_dir is not None
            else default_telemetry_dir(run_id)
        )
        self._series: dict[str, _SeriesMeta] = {}
        self._agents: dict[str, TelemetryAgent] = {}
        self._rows: list[dict[str, float | None]] = []
        self._closed = False
        self._created_utc = datetime.now(timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"
        )
        self._wall_t0 = time.time()
        self._csv_path: pathlib.Path | None = None
        self._manifest_path: pathlib.Path | None = None

    @property
    def csv_path(self) -> pathlib.Path:
        """Resolved path of ``{csv_basename}.csv``."""
        return self.output_dir / f"{self.csv_basename}.csv"

    @property
    def manifest_path(self) -> pathlib.Path:
        """Resolved path of ``{csv_basename}.json`` manifest."""
        return self.output_dir / f"{self.csv_basename}.json"

    def register_agent(
        self,
        name: str,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> TelemetryAgent:
        """Register a named agent and return its registration handle."""
        if not self.enabled:
            return TelemetryAgent(
                name=name, _session=self, metadata=dict(metadata or {})
            )
        validate_agent_name(name)
        if name in self._agents:
            raise ValueError(f"agent already registered: {name!r}")
        agent = TelemetryAgent(
            name=name,
            _session=self,
            kind="agent",
            metadata=dict(metadata or {}),
        )
        self._agents[name] = agent
        return agent

    def register_simulator(
        self,
        name: str = "sim",
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> TelemetryAgent:
        """Register a simulator-as-agent (only when metadata is useful)."""
        if not self.enabled:
            return TelemetryAgent(
                name=name,
                _session=self,
                kind="simulator",
                metadata=dict(metadata or {}),
            )
        validate_agent_name(name)
        if name in self._agents:
            raise ValueError(f"agent already registered: {name!r}")
        agent = TelemetryAgent(
            name=name,
            _session=self,
            kind="simulator",
            metadata=dict(metadata or {}),
        )
        self._agents[name] = agent
        return agent

    def register_series(
        self,
        series_id: str,
        *,
        unit: str,
        derived: bool = False,
    ) -> None:
        """Register a fully-qualified series id (advanced / tests)."""
        if not self.enabled:
            return
        validate_series_id(series_id)
        if series_id in self._series:
            raise ValueError(f"series already registered: {series_id!r}")
        parsed: SeriesId = parse_series_id(series_id)
        self._series[series_id] = _SeriesMeta(
            series_id=series_id,
            unit=unit,
            quantity=parsed.quantity,
            frame=parsed.frame,
            component=parsed.component,
            agent=parsed.agent,
            derived=derived,
        )

    def record(
        self,
        t: float,
        values: Mapping[str, float],
        *,
        t_wall: float | None = None,
        tick: int | None = None,
    ) -> None:
        """Append one sample row keyed by registered series ids."""
        if not self.enabled or self._closed:
            return
        if not self._series:
            raise RuntimeError(
                "TelemetrySession.record called before any series registration"
            )
        row: dict[str, float | None] = {
            "t": float(t),
            "t_wall": (
                float(t_wall)
                if t_wall is not None
                else float(time.time() - self._wall_t0)
            ),
        }
        if tick is not None:
            row["tick"] = float(tick)
        unknown = [k for k in values if k not in self._series]
        if unknown:
            if self.strict:
                raise KeyError(
                    "unknown telemetry series (register first): "
                    + ", ".join(sorted(unknown))
                )
            # Drop unknown keys in non-strict mode.
            values = {k: v for k, v in values.items() if k in self._series}
        for series_id in self._series:
            if series_id in values:
                row[series_id] = float(values[series_id])
            else:
                row[series_id] = None
        self._rows.append(row)

    def close(self) -> pathlib.Path | None:
        """Flush CSV + manifest. Idempotent. Returns CSV path when enabled."""
        if not self.enabled:
            self._closed = True
            return None
        if self._closed:
            return self._csv_path
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._write_csv()
        self._write_manifest()
        self._closed = True
        return self._csv_path

    def _column_order(self) -> list[str]:
        cols = ["t", "t_wall"]
        if any("tick" in row for row in self._rows):
            cols.append("tick")
        cols.extend(sorted(self._series))
        return cols

    def _write_csv(self) -> None:
        path = self.csv_path
        columns = self._column_order()
        if len(columns) != len(set(columns)):
            raise RuntimeError("duplicate CSV headers")
        with open(path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh, lineterminator="\n")
            writer.writerow(columns)
            for row in self._rows:
                out: list[str] = []
                for col in columns:
                    value = row.get(col)
                    if value is None:
                        out.append("")
                    else:
                        out.append(format(float(value), ".9g"))
                writer.writerow(out)
        self._csv_path = path

    def _write_manifest(self) -> None:
        frames_used = sorted({m.frame for m in self._series.values()})
        agents = sorted(self._agents)
        series_payload: dict[str, Any] = {}
        for sid, meta in sorted(self._series.items()):
            entry: dict[str, Any] = {
                "unit": meta.unit,
                "quantity": meta.quantity,
                "frame": meta.frame,
                "component": meta.component,
            }
            if meta.derived:
                entry["derived"] = True
            series_payload[sid] = entry
        agents_meta = {
            name: {
                "kind": agent.kind,
                "metadata": agent.metadata,
            }
            for name, agent in sorted(self._agents.items())
        }
        payload: dict[str, Any] = {
            "schema_version": _SCHEMA_VERSION,
            "format": _FORMAT,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "created_utc": self._created_utc,
            "time_column": "t",
            "time_unit": "s",
            "agents": agents,
            "agents_meta": agents_meta,
            "frames_used": frames_used,
            "series": series_payload,
            "row_count": len(self._rows),
            "plotjuggler": {
                "recommended_time_axis": "t",
                "delimiter": ",",
            },
        }
        if self.dt_nominal_s is not None:
            payload["dt_nominal_s"] = float(self.dt_nominal_s)
        path = self.manifest_path
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
            fh.write("\n")
        self._manifest_path = path


def merge_values(*parts: Mapping[str, float]) -> dict[str, float]:
    """Merge several value dicts (later keys overwrite)."""
    out: dict[str, float] = {}
    for part in parts:
        out.update({k: float(v) for k, v in part.items()})
    return out


def se2_pose_values(
    agent: str,
    pose: tuple[float, float, float] | Sequence[float],
) -> dict[str, float]:
    """Build ENU position + yaw values for an SE(2) pose ``(x, y, yaw)``."""
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    return {
        f"{agent}.position_enu.x": x,
        f"{agent}.position_enu.y": y,
        f"{agent}.orientation_enu.yaw": yaw,
    }


def register_se2_mobile_agent(
    session: TelemetrySession,
    name: str,
    *,
    with_commands: bool = True,
    with_rates: bool = True,
    with_path_error: bool = False,
    metadata: Mapping[str, Any] | None = None,
) -> TelemetryAgent:
    """Register a Dubins-style mobile agent with the common SE(2) field set."""
    agent = session.register_agent(name, metadata=metadata)
    agent.register("position", frame="enu", components=("x", "y"), unit="m")
    agent.register("orientation", frame="enu", components=("yaw",), unit="rad")
    if with_rates:
        agent.register("velocity", frame="body", components=("x",), unit="m/s")
        agent.register("omega", frame="body", components=("z",), unit="rad/s")
    if with_commands:
        agent.register(
            "cmd_velocity", frame="ctrl", components=("val",), unit="m/s"
        )
        agent.register(
            "cmd_omega", frame="ctrl", components=("val",), unit="rad/s"
        )
    if with_path_error:
        agent.register(
            "cross_track", frame="map", components=("val",), unit="m"
        )
        agent.register("progress", frame="map", components=("val",), unit="m")
    return agent


def register_arm_agent(
    session: TelemetrySession,
    name: str,
    joint_names: Iterable[str],
    *,
    with_ee: bool = True,
    metadata: Mapping[str, Any] | None = None,
) -> TelemetryAgent:
    """Register a manipulator agent with joint positions (+ optional EE XYZ)."""
    agent = session.register_agent(name, metadata=metadata)
    joints = tuple(joint_names)
    if joints:
        agent.register(
            "position",
            frame="joint",
            components=joints,
            unit="rad",
        )
    if with_ee:
        agent.register(
            "ee_position", frame="enu", components=("x", "y", "z"), unit="m"
        )
    return agent
