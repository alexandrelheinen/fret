"""Unit tests for controller ROS dispatch helpers (T10-06)."""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from fret.control.controller_node import (
    compute_tracking_command,
    controller_is_halted,
    controller_update_rate_hz,
    make_controller_node,
)
from fret.control.controller_ppp import PPPControllerState
from fret.control.kinematics import Kinematics


def _ppp_config() -> str:
    return str(
        pathlib.Path(__file__).resolve().parents[1]
        / "src"
        / "fret"
        / "config"
        / "controllers"
        / "ppp.yml"
    )


def test_controller_update_rate_ppp() -> None:
    logic = make_controller_node("ppp", _ppp_config())
    assert controller_update_rate_hz(logic, "ppp") == 50.0


def test_compute_tracking_command_ppp(tmp_path: pathlib.Path) -> None:
    import yaml

    config_file = tmp_path / "ppp.yml"
    config_file.write_text(
        yaml.dump(
            {
                "/**": {
                    "ros__parameters": {
                        "kp": 1.5,
                        "max_joint_velocity": [3.0, 3.0, 1.5],
                        "fault_threshold": 1.0,
                        "update_rate": 50.0,
                    }
                }
            }
        )
    )
    logic = make_controller_node("ppp", config_file)
    kin = Kinematics("ppp")
    logic.set_trajectory([np.zeros(3), np.array([0.1, 0.0, 0.0])])
    logic._trajectory_index = 1
    q_dot = compute_tracking_command(logic, "ppp", kin, np.zeros(3))
    assert q_dot.shape == (3,)
    assert q_dot[0] > 0.0


def test_controller_is_halted_ppp() -> None:
    logic = make_controller_node("ppp", _ppp_config())
    logic._enter_halted()
    assert controller_is_halted(logic, "ppp") is True
    assert logic.state == PPPControllerState.HALTED
