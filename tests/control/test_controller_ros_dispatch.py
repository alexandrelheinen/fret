"""Unit tests for controller ROS dispatch helpers (T10-06)."""

from __future__ import annotations

import pathlib
from typing import Any

import numpy as np
import pytest
import yaml

from fret.config_loader import load_ros_parameters_yaml
from fret.control.controller_node import (
    compute_tracking_command,
    controller_is_halted,
    controller_update_rate_hz,
    make_controller_node,
)
from fret.control.kinematics import Kinematics


def _scara_config_path() -> pathlib.Path:
    return (
        pathlib.Path(__file__).resolve().parents[2]
        / "src"
        / "fret"
        / "config"
        / "controllers"
        / "jacobian.yml"
    )


def _bundled_controller_params() -> dict[str, Any]:
    return dict(load_ros_parameters_yaml(_scara_config_path()))


def _write_scara_controller_yaml(
    path: pathlib.Path,
    **overrides: Any,
) -> None:
    params = _bundled_controller_params()
    params.update(overrides)
    path.write_text(
        yaml.dump({"/**": {"ros__parameters": params}}),
        encoding="utf-8",
    )


def test_controller_update_rate_scara() -> None:
    logic = make_controller_node("scara", _scara_config_path())
    assert controller_update_rate_hz(logic, "scara") == 50.0


def test_compute_tracking_command_scara(tmp_path: pathlib.Path) -> None:
    config_file = tmp_path / "jacobian.yml"
    _write_scara_controller_yaml(config_file, kp=1.5, fault_threshold=1.0)
    logic = make_controller_node("scara", config_file)
    kin = Kinematics("scara")
    logic.set_trajectory([np.zeros(3), np.array([0.1, 0.0, 0.0])])
    logic._trajectory_index = 1
    q_dot = compute_tracking_command(logic, "scara", kin, np.zeros(3))
    assert q_dot.shape == (3,)


def test_controller_is_halted_scara() -> None:
    logic = make_controller_node("scara", _scara_config_path())
    logic._enter_halted()
    assert controller_is_halted(logic, "scara") is True


def test_make_controller_node_unknown_raises() -> None:
    with pytest.raises(ValueError, match="Unknown controller model"):
        make_controller_node("dubins", _scara_config_path())
