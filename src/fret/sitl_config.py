"""Pure-Python helpers for SITL launch configuration (T10-06).

Resolves backend-specific asset paths so launch files and unit tests share
the same routing rules without importing ROS at module import time.
"""

from __future__ import annotations

import pathlib
from typing import Any

import yaml

_PPP_WAREHOUSE_SCENARIO = "ppp_warehouse"
_DUBINS_RACE_SCENARIO = "dubins_race"
_PPP_MODEL = "ppp"
_DUBINS_MODEL = "dubins"
_MUJOCO_BACKEND = "mujoco"


def controller_config_relative(model: str) -> str:
    """Return package-relative controller YAML path for a robot model.

    Args:
        model: Robot model name (``ppp``, ``scara``, …).

    Returns:
        Path relative to the ``fret`` package share root.
    """
    if model == _PPP_MODEL:
        return "config/controllers/ppp.yml"
    if model == _DUBINS_MODEL:
        return "config/controllers/dubins.yml"
    return "config/controllers/jacobian.yml"


def perception_config_relative(scenario: str) -> str:
    """Return package-relative perception YAML for a scenario.

    Args:
        scenario: Scenario stem (e.g. ``ppp_warehouse``).

    Returns:
        Path relative to the ``fret`` package share root.
    """
    if scenario == _PPP_WAREHOUSE_SCENARIO:
        return "config/perception_ppp_warehouse.yaml"
    if scenario == _DUBINS_RACE_SCENARIO:
        return "config/perception.yaml"
    return "config/perception.yaml"


def mujoco_sim_config_relative() -> str:
    """Return package-relative MuJoCo bridge YAML path."""
    return "config/simulation/mujoco.yml"


def uses_mujoco_backend(backend: str) -> bool:
    """Return ``True`` when the SITL backend is MuJoCo."""
    return backend == _MUJOCO_BACKEND


def is_ppp_warehouse_launch(model: str, scenario: str) -> bool:
    """Return ``True`` for the v1.0 PPP warehouse product launch tuple."""
    return model == _PPP_MODEL and scenario == _PPP_WAREHOUSE_SCENARIO


def is_dubins_race_launch(model: str, scenario: str) -> bool:
    """Return ``True`` for the v1.1 Dubins race product launch tuple."""
    return model == _DUBINS_MODEL and scenario == _DUBINS_RACE_SCENARIO


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
