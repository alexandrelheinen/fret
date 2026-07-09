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
from fret.control.controller_ppp import PPPControllerState
from fret.control.kinematics import Kinematics


def _ppp_config_path() -> pathlib.Path:
    return (
        pathlib.Path(__file__).resolve().parents[2]
        / "src"
        / "fret"
        / "config"
        / "controllers"
        / "ppp.yml"
    )


def _bundled_controller_params() -> dict[str, Any]:
    return dict(load_ros_parameters_yaml(_ppp_config_path()))


def _write_ppp_controller_yaml(
    path: pathlib.Path,
    **overrides: Any,
) -> None:
    params = _bundled_controller_params()
    params.update(overrides)
    path.write_text(
        yaml.dump({"/**": {"ros__parameters": params}}),
        encoding="utf-8",
    )


def test_controller_update_rate_ppp() -> None:
    logic = make_controller_node("ppp", _ppp_config_path())
    assert controller_update_rate_hz(logic, "ppp") == 50.0


def test_compute_tracking_command_ppp(tmp_path: pathlib.Path) -> None:
    config_file = tmp_path / "ppp.yml"
    _write_ppp_controller_yaml(config_file, kp=1.5, fault_threshold=1.0)
    logic = make_controller_node("ppp", config_file)
    kin = Kinematics("ppp")
    logic.set_trajectory([np.zeros(3), np.array([0.1, 0.0, 0.0])])
    logic._trajectory_index = 1
    q_dot = compute_tracking_command(logic, "ppp", kin, np.zeros(3))
    assert q_dot.shape == (3,)
    assert q_dot[0] > 0.0


def test_controller_is_halted_ppp() -> None:
    logic = make_controller_node("ppp", _ppp_config_path())
    logic._enter_halted()
    assert controller_is_halted(logic, "ppp") is True
    assert logic.state == PPPControllerState.HALTED
