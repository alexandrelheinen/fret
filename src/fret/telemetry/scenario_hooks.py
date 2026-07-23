"""Helpers to attach telemetry to scenario runners."""

from __future__ import annotations

import pathlib
from typing import Any, Mapping, Sequence

from fret.telemetry.session import (
    TelemetrySession,
    make_run_id,
    register_arm_agent,
    register_se2_mobile_agent,
    se2_pose_values,
    telemetry_enabled_from_env,
)


def resolve_telemetry_enabled(enabled: bool | None) -> bool:
    """Resolve opt-in flag (explicit override, else env)."""
    if enabled is not None:
        return bool(enabled)
    return telemetry_enabled_from_env()


def open_scenario_telemetry(
    scenario_id: str,
    *,
    enabled: bool | None = None,
    output_dir: pathlib.Path | None = None,
    csv_basename: str | None = None,
    dt_nominal_s: float | None = None,
) -> TelemetrySession | None:
    """Create an enabled :class:`TelemetrySession` or return ``None``."""
    if not resolve_telemetry_enabled(enabled):
        return None
    run_id = make_run_id(scenario_id)
    return TelemetrySession(
        run_id=run_id,
        scenario_id=scenario_id,
        enabled=True,
        output_dir=output_dir,
        dt_nominal_s=dt_nominal_s,
        csv_basename=csv_basename or "telemetry",
    )


def setup_dubins_telemetry(
    session: TelemetrySession,
    *,
    physics_mode: bool,
) -> None:
    """Register TB3 race agents + optional simulator metadata."""
    register_se2_mobile_agent(
        session,
        "tb3_rrt",
        with_commands=True,
        with_rates=True,
        with_path_error=True,
        metadata={"role": "rrt_star"},
    )
    register_se2_mobile_agent(
        session,
        "tb3_sst",
        with_commands=True,
        with_rates=True,
        with_path_error=True,
        metadata={"role": "sst"},
    )
    register_se2_mobile_agent(
        session,
        "tb3_dummy",
        with_commands=True,
        with_rates=True,
        with_path_error=False,
        metadata={"role": "dummy_foil"},
    )
    session.register_simulator(
        "mujoco",
        metadata={
            "physics_mode": bool(physics_mode),
            "scenario": "dubins_race",
        },
    )
    # Scalar flag so PlotJuggler can plot physics vs kinematic runs.
    sim = session._agents["mujoco"]  # noqa: SLF001
    sim.register("physics_mode", frame="ctrl", components=("val",), unit="1")


def dubins_agent_values(
    agent: str,
    *,
    pose: tuple[float, float, float] | Sequence[float],
    speed: float,
    turn_rate: float,
    cmd_speed: float,
    cmd_omega: float,
    cross_track: float | None = None,
    progress: float | None = None,
) -> dict[str, float]:
    """Build one agent's Dubins sample dict."""
    values = se2_pose_values(agent, pose)
    values[f"{agent}.velocity_body.x"] = float(speed)
    values[f"{agent}.omega_body.z"] = float(turn_rate)
    values[f"{agent}.cmd_velocity_ctrl.val"] = float(cmd_speed)
    values[f"{agent}.cmd_omega_ctrl.val"] = float(cmd_omega)
    if cross_track is not None:
        values[f"{agent}.cross_track_map.val"] = float(cross_track)
    if progress is not None:
        values[f"{agent}.progress_map.val"] = float(progress)
    return values


def setup_arm_telemetry(
    session: TelemetrySession,
    agent_name: str,
    joint_names: Sequence[str],
    *,
    physics_mode: bool = True,
) -> list[str]:
    """Register manipulator joints (lowercased) + EE position.

    Returns:
        Component ids used for ``position_joint.*`` (same order as joints).
    """
    components = [str(n).lower().replace("-", "_") for n in joint_names]
    register_arm_agent(
        session,
        agent_name,
        components,
        with_ee=True,
        metadata={"joints": list(joint_names)},
    )
    session.register_simulator(
        "mujoco",
        metadata={"physics_mode": bool(physics_mode)},
    )
    sim = session._agents["mujoco"]  # noqa: SLF001
    sim.register("physics_mode", frame="ctrl", components=("val",), unit="1")
    return components


def arm_sample_values(
    agent_name: str,
    joint_components: Sequence[str],
    q_arm: Sequence[float] | Any,
    ee_pos: Sequence[float] | Any | None,
    *,
    physics_mode: bool = True,
) -> dict[str, float]:
    """Build arm joint + EE telemetry values for one tick."""
    values: dict[str, float] = {
        "mujoco.physics_mode_ctrl.val": 1.0 if physics_mode else 0.0,
    }
    for comp, q in zip(joint_components, q_arm, strict=False):
        values[f"{agent_name}.position_joint.{comp}"] = float(q)
    if ee_pos is not None and len(ee_pos) >= 3:
        values[f"{agent_name}.ee_position_enu.x"] = float(ee_pos[0])
        values[f"{agent_name}.ee_position_enu.y"] = float(ee_pos[1])
        values[f"{agent_name}.ee_position_enu.z"] = float(ee_pos[2])
    return values


def close_telemetry(
    session: TelemetrySession | None,
) -> tuple[pathlib.Path | None, pathlib.Path | None]:
    """Close a session and return ``(csv_path, manifest_path)``."""
    if session is None:
        return None, None
    csv_path = session.close()
    manifest = session.manifest_path if csv_path is not None else None
    return csv_path, manifest
