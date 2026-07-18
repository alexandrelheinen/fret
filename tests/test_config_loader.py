"""Tests for fret.config_loader."""

from __future__ import annotations

import pathlib

import pytest

from fret.config_loader import (
    load_algorithm_config,
    load_scenario_bundle,
    planning_config_for_model,
    resolve_obstacle_file,
)


def test_planning_config_for_model() -> None:
    assert planning_config_for_model("dubins") == "planning/dubins.yml"
    import pytest

    with pytest.raises(ValueError, match="No planning config"):
        planning_config_for_model("unknown_model")


def test_load_dubins_planning_config() -> None:
    cfg = load_algorithm_config("planning/dubins.yml")
    assert "obstacle_file" in cfg


def test_scenario_bundle_resolves_obstacle_file() -> None:
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "scenarios"
        / "dubins_race.yml"
    )
    bundle = load_scenario_bundle(path)
    obstacle = resolve_obstacle_file(bundle.planning)
    assert obstacle.name == "dubins_race_obstacles.yml"
