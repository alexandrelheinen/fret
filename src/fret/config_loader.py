"""YAML configuration loader for FRET algorithm parameters.

Tunable planning, control, and grasp parameters live under ``config/`` and are
loaded at runtime.  Source modules must not embed algorithm defaults; change
behavior by editing YAML only.

Scenario files reference bundled configs via ``planning_config`` and optional
``grasp_config`` keys.  Per-scenario overrides may be nested under ``planning``
or ``grasp`` in the scenario YAML.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Any

import yaml

from fret.sitl_config import load_scenario_parameters, resolve_package_file


def load_yaml_file(path: str | pathlib.Path) -> dict[str, Any]:
    """Load a plain YAML mapping from disk.

    Args:
        path: File path.

    Returns:
        Top-level mapping.

    Raises:
        FileNotFoundError: When the file is missing.
        ValueError: When the root is not a mapping.
    """
    file_path = pathlib.Path(path)
    if not file_path.is_file():
        raise FileNotFoundError(f"Config file not found: {file_path}")

    with file_path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root (expected mapping): {file_path}")
    return data


def load_ros_parameters_yaml(path: str | pathlib.Path) -> dict[str, Any]:
    """Load the first ``/**/ros__parameters`` block from a ROS-style YAML file."""
    data = load_yaml_file(path)
    for section in data.values():
        if isinstance(section, dict):
            params = section.get("ros__parameters")
            if isinstance(params, dict):
                return dict(params)
    raise ValueError(f"No ros__parameters block in {path}")


def load_algorithm_config(relative_path: str) -> dict[str, Any]:
    """Load a plain algorithm config under ``config/``."""
    path = resolve_package_file("config", *relative_path.split("/"))
    return load_yaml_file(path)


def algorithm_config_path(relative_path: str) -> pathlib.Path:
    """Resolve ``config/<relative_path>`` from share or source tree."""
    return resolve_package_file("config", *relative_path.split("/"))


def require_key(
    mapping: dict[str, Any],
    key: str,
    *,
    context: str,
) -> Any:
    """Return ``mapping[key]`` or raise ``KeyError`` with context."""
    if key not in mapping:
        raise KeyError(f"Missing required key {key!r} in {context}")
    return mapping[key]


def require_keys(
    mapping: dict[str, Any],
    keys: tuple[str, ...],
    *,
    context: str,
) -> None:
    """Raise ``KeyError`` when any required key is absent."""
    missing = [key for key in keys if key not in mapping]
    if missing:
        joined = ", ".join(repr(key) for key in missing)
        raise KeyError(f"Missing required keys [{joined}] in {context}")


def merge_configs(
    base: dict[str, Any],
    override: dict[str, Any] | None,
) -> dict[str, Any]:
    """Deep-merge ``override`` into a copy of ``base``."""
    if not override:
        return dict(base)

    merged: dict[str, Any] = dict(base)
    for key, value in override.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            merged[key] = merge_configs(merged[key], value)
        else:
            merged[key] = value
    return merged


@dataclass(frozen=True)
class ScenarioBundle:
    """Resolved scenario parameters plus referenced algorithm configs."""

    parameters: dict[str, Any]
    planning: dict[str, Any]
    grasp: dict[str, Any] | None


def load_scenario_bundle(path: str | pathlib.Path) -> ScenarioBundle:
    """Load scenario ROS parameters and referenced planning/grasp configs.

    Args:
        path: Scenario YAML under ``config/scenarios/``.

    Returns:
        Flat scenario parameters plus merged planning and optional grasp dicts.

    Raises:
        KeyError: When ``planning_config`` is missing from the scenario.
    """
    params = load_scenario_parameters(path)
    context = f"scenario {pathlib.Path(path).name}"

    planning_rel = str(require_key(params, "planning_config", context=context))
    planning = load_algorithm_config(planning_rel)
    planning = merge_configs(planning, params.get("planning"))

    grasp: dict[str, Any] | None = None
    grasp_rel = params.get("grasp_config")
    if grasp_rel is not None:
        grasp = load_algorithm_config(str(grasp_rel))
        grasp = merge_configs(grasp, params.get("grasp"))

    return ScenarioBundle(parameters=params, planning=planning, grasp=grasp)


def resolve_obstacle_file(planning: dict[str, Any]) -> pathlib.Path:
    """Return the obstacle YAML referenced by a planning config."""
    rel = str(
        require_key(planning, "obstacle_file", context="planning config")
    )
    return resolve_package_file("config", *rel.split("/"))


def planning_config_for_model(model: str) -> str:
    """Return the default planning config relative path for a robot model."""
    if model == "dubins":
        return "planning/dubins.yml"
    return "planning/scara.yml"
