"""Unit tests for SITL launch configuration helpers (T10-06)."""

from __future__ import annotations

import pathlib

import pytest

from fret.config_loader import load_scenario_bundle
from fret.sitl_config import (
    controller_config_relative,
    load_scenario_parameters,
    mujoco_sim_config_relative,
    perception_config_relative,
    physics_controller_config_relative,
    resolve_package_file,
)


def test_controller_config_relative_dubins() -> None:
    assert (
        controller_config_relative("dubins") == "config/controllers/dubins.yml"
    )


def test_physics_controller_config_relative_unknown_raises() -> None:
    with pytest.raises(ValueError, match="No physics controller profile"):
        physics_controller_config_relative("dubins")


def test_controller_config_relative_scara() -> None:
    assert (
        controller_config_relative("scara")
        == "config/controllers/jacobian.yml"
    )


def test_perception_config_relative_dubins_race() -> None:
    assert (
        perception_config_relative("dubins_race") == "config/perception.yaml"
    )


def test_perception_config_relative_default() -> None:
    assert (
        perception_config_relative("static_reach") == "config/perception.yaml"
    )


def test_mujoco_sim_config_relative() -> None:
    assert mujoco_sim_config_relative() == "config/simulation/mujoco.yml"


def test_load_dubins_race_scenario() -> None:
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "scenarios"
        / "dubins_race.yml"
    )
    params = load_scenario_parameters(path)
    assert params["scenario_id"] == "dubins_race"
    assert params["model"] == "dubins"
    assert params["backend"] == "mujoco"
    assert params["planning_config"] == "planning/dubins.yml"


def test_load_dubins_race_scenario_bundle() -> None:
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "scenarios"
        / "dubins_race.yml"
    )
    bundle = load_scenario_bundle(path)
    assert "obstacle_file" in bundle.planning


def test_load_scenario_missing_file() -> None:
    with pytest.raises(FileNotFoundError):
        load_scenario_parameters("/nonexistent/scenario.yml")


def test_resolve_package_file_from_source_tree() -> None:
    path = resolve_package_file("config", "scenarios", "dubins_race.yml")
    assert path.name == "dubins_race.yml"
    assert path.is_file()


def test_mjcf_path_dubins_race() -> None:
    from fret.sitl_config import mjcf_path

    path = mjcf_path("dubins", "dubins_race")
    assert path.name == "dubins_race.xml"
    assert path.is_file()
