"""Unit tests for SITL launch configuration helpers (T10-06)."""

from __future__ import annotations

import pathlib

import pytest

from fret.sitl_config import (
    controller_config_relative,
    is_ppp_warehouse_launch,
    load_scenario_parameters,
    mujoco_sim_config_relative,
    perception_config_relative,
    uses_mujoco_backend,
)


def test_controller_config_relative_ppp() -> None:
    assert controller_config_relative("ppp") == "config/controllers/ppp.yml"


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


def test_uses_mujoco_backend() -> None:
    assert uses_mujoco_backend("mujoco") is True
    assert uses_mujoco_backend("gazebo") is False


def test_is_ppp_warehouse_launch() -> None:
    assert is_ppp_warehouse_launch("ppp", "ppp_warehouse") is True
    assert is_ppp_warehouse_launch("scara", "ppp_warehouse") is False


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
    assert params["start_configuration"] == [2.0, 1.0, 2.4]
    assert params["goal_configuration"] == [10.5, 3.2, 2.4]
    assert params["planning_timeout"] == 30.0


def test_load_scenario_missing_file() -> None:
    with pytest.raises(FileNotFoundError):
        load_scenario_parameters("/nonexistent/scenario.yml")
