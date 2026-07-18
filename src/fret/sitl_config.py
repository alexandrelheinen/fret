"""Pure-Python helpers for SITL launch configuration (T10-06).

Resolves backend-specific asset paths so launch files and unit tests share
the same routing rules without importing ROS at module import time.
"""

from __future__ import annotations

import pathlib
from typing import Any

import yaml

_DUBINS_RACE_SCENARIO = "dubins_race"
_DUBINS_MODEL = "dubins"
_DIFFDRIVE_MODEL = "diffdrive"


def controller_config_relative(model: str) -> str:
    """Return package-relative controller YAML path for a robot model.

    Args:
        model: Robot model name (``dubins``, ``open_manipulator_x``, …).

    Returns:
        Path relative to the ``fret`` package share root.
    """
    if model == _DUBINS_MODEL:
        return "config/controllers/dubins.yml"
    if model in {"open_manipulator_x", "omx"}:
        return "config/controllers/open_manipulator_x.yml"
    raise ValueError(f"No controller config for model: {model!r}")


def physics_controller_config_relative(model: str) -> str:
    """Return package-relative physics SITL controller YAML for a model.

    Args:
        model: Robot model name.

    Returns:
        Path relative to the ``fret`` package share root.

    Raises:
        ValueError: When no physics profile exists for the model.
    """
    raise ValueError(f"No physics controller profile for model: {model!r}")


def perception_config_relative(scenario: str) -> str:
    """Return package-relative perception YAML for a scenario.

    Args:
        scenario: Scenario stem (e.g. ``dubins_race``).

    Returns:
        Path relative to the ``fret`` package share root.
    """
    if scenario == _DUBINS_RACE_SCENARIO:
        return "config/perception.yaml"
    return "config/perception.yaml"


def mujoco_sim_config_relative() -> str:
    """Return package-relative MuJoCo bridge YAML path."""
    return "config/simulation/mujoco.yml"


def package_source_root() -> pathlib.Path:
    """Return the on-disk ``fret`` Python package root (dev / editable installs)."""
    return pathlib.Path(__file__).resolve().parent


def package_share_root() -> pathlib.Path:
    """Return the installed ROS share directory for ``fret`` when available."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return pathlib.Path(get_package_share_directory("fret"))
    except Exception:
        return package_source_root()


def resolve_package_file(*parts: str) -> pathlib.Path:
    """Resolve a file under ``share/fret`` or the source package tree.

    Args:
        *parts: Path segments relative to the package root, e.g.
            ``("config", "scenarios", "dubins_race.yml")``.

    Returns:
        First existing match in share, then source tree.

    Raises:
        FileNotFoundError: If the file cannot be located.
    """
    seen: set[pathlib.Path] = set()
    for root in (package_share_root(), package_source_root()):
        if root in seen:
            continue
        seen.add(root)
        candidate = root.joinpath(*parts)
        if candidate.is_file():
            return candidate
    joined = "/".join(parts)
    raise FileNotFoundError(f"Package file not found: {joined}")


def scenario_config_path(stem: str) -> pathlib.Path:
    """Return ``config/scenarios/<stem>.yml`` from share or source."""
    return resolve_package_file("config", "scenarios", f"{stem}.yml")


def controller_config_path(model: str) -> pathlib.Path:
    """Return the controller YAML for a robot model."""
    rel = controller_config_relative(model)
    return resolve_package_file(*rel.split("/"))


def physics_controller_config_path(model: str) -> pathlib.Path:
    """Return the physics SITL controller YAML for a robot model."""
    rel = physics_controller_config_relative(model)
    return resolve_package_file(*rel.split("/"))


def mjcf_path(model: str, scenario: str) -> pathlib.Path:
    """Return the MJCF scene file for a model/scenario pair."""
    if model == _DUBINS_MODEL and scenario in {
        _DUBINS_RACE_SCENARIO,
        "dubins",
    }:
        return resolve_package_file("mjcf", "dubins_race.xml")
    if model == _DIFFDRIVE_MODEL and scenario in {
        "diffdrive_unit",
        "diffdrive",
    }:
        return resolve_package_file("mjcf", "diffdrive_unit.xml")
    if model in {"turtlebot3", "tb3"} and scenario in {
        "turtlebot3_unit",
        "turtlebot3",
        "tb3",
    }:
        return resolve_package_file("mjcf", "turtlebot3_unit.xml")
    if model in {"open_manipulator_x", "omx"} and scenario in {
        "omx_reach",
        "omx_tabletop",
        "open_manipulator_x",
    }:
        from fret.mjcf.omx import ensure_omx_tabletop_mjcf

        return ensure_omx_tabletop_mjcf()
    if model in {"open_manipulator_x", "omx"} and scenario in {
        "omx_pick_place",
        "pick_place",
    }:
        from fret.mjcf.omx import ensure_omx_pick_place_mjcf

        return ensure_omx_pick_place_mjcf()
    raise ValueError(
        f"Unsupported model/scenario combination: model={model!r}, "
        f"scenario={scenario!r}"
    )


def load_scenario_parameters(path: str | pathlib.Path) -> dict[str, Any]:
    """Load ``ros__parameters`` from a scenario YAML file.

    Args:
        path: Path to ``config/scenarios/<scenario>.yml``.

    Returns:
        Flat parameter dict from the first ``/**/ros__parameters`` block.

    Raises:
        FileNotFoundError: If the scenario file does not exist.
        ValueError: If no ``ros__parameters`` section is present.
    """
    scenario_path = pathlib.Path(path)
    if not scenario_path.is_file():
        raise FileNotFoundError(f"Scenario file not found: {scenario_path}")

    with scenario_path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    if not isinstance(data, dict):
        raise ValueError(f"Invalid scenario YAML: {scenario_path}")

    for section in data.values():
        if isinstance(section, dict):
            params = section.get("ros__parameters")
            if isinstance(params, dict):
                return dict(params)

    raise ValueError(f"No ros__parameters block in {scenario_path}")
