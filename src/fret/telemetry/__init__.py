"""Opt-in PlotJuggler CSV telemetry (FR-SIM-12).

See ``docs/modules/telemetry.md`` for the normative naming grammar and
CSV contract.
"""

from __future__ import annotations

from fret.telemetry.layout_paths import (
    layout_path_for_scenario,
    layouts_dir,
    load_layout_index,
)
from fret.telemetry.naming import (
    FRAMES,
    SeriesId,
    build_series_id,
    parse_series_id,
    validate_series_id,
)
from fret.telemetry.scenario_hooks import (
    open_scenario_telemetry,
    resolve_telemetry_enabled,
)
from fret.telemetry.session import (
    TelemetryAgent,
    TelemetrySession,
    default_telemetry_dir,
    make_run_id,
    merge_values,
    register_arm_agent,
    register_se2_mobile_agent,
    se2_pose_values,
    telemetry_enabled_from_env,
)

__all__ = [
    "FRAMES",
    "SeriesId",
    "TelemetryAgent",
    "TelemetrySession",
    "build_series_id",
    "default_telemetry_dir",
    "layout_path_for_scenario",
    "layouts_dir",
    "load_layout_index",
    "make_run_id",
    "merge_values",
    "open_scenario_telemetry",
    "parse_series_id",
    "register_arm_agent",
    "register_se2_mobile_agent",
    "resolve_telemetry_enabled",
    "se2_pose_values",
    "telemetry_enabled_from_env",
    "validate_series_id",
]
