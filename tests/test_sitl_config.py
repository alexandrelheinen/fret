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


def test_controller_config_relative_ppp() -> None:
    assert controller_config_relative("ppp") == "config/controllers/ppp.yml"


def test_physics_controller_config_relative_ppp() -> None:
    assert (
        physics_controller_config_relative("ppp")
        == "config/controllers/ppp_physics.yml"
    )


def test_physics_controller_config_relative_unknown_raises() -> None:
    with pytest.raises(ValueError, match="No physics controller profile"):
        physics_controller_config_relative("dubins")


def test_controller_config_relative_scara() -> None:
    assert (
        controller_config_relative("scara")
        == "config/controllers/jacobian.yml"
    )


def test_perception_config_relative_ppp_warehouse() -> None:
    assert (
        perception_config_relative("ppp_warehouse")
        == "config/perception_ppp_warehouse.yaml"
    )


def test_perception_config_relative_default() -> None:
    assert (
        perception_config_relative("static_reach") == "config/perception.yaml"
    )


def test_mujoco_sim_config_relative() -> None:
    assert mujoco_sim_config_relative() == "config/simulation/mujoco.yml"


def test_load_ppp_warehouse_scenario() -> None:
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "scenarios"
        / "ppp_warehouse.yml"
    )
    params = load_scenario_parameters(path)
    assert params["scenario_id"] == "ppp_warehouse"
    assert params["model"] == "ppp"
    assert params["backend"] == "mujoco"
    assert params["planning_config"] == "planning/ppp.yml"
    assert params["grasp_config"] == "grasp/ppp_warehouse.yml"
    assert params["start_configuration"] == [2.0, 1.0, 2.4]
    assert params["goal_configuration"] == [10.5, 1.2, 0.59]
    assert params["collision_backend"] == "analytic"
    assert params["planner_algorithm"] == "rrt_star"
    assert params["plan_include_cargo"] is True
    assert params["planning_timeout"] == 30.0


def test_load_ppp_warehouse_scenario_bundle() -> None:
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "scenarios"
        / "ppp_warehouse.yml"
    )
    bundle = load_scenario_bundle(path)
    assert bundle.planning["contact_radius"] == pytest.approx(0.015)
    assert bundle.grasp is not None
    assert bundle.grasp["capture_radius"] == pytest.approx(0.45)


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
