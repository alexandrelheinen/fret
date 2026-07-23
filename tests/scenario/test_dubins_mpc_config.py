"""Unit tests for Dubins race MPC config wiring (progress-first fields)."""

from __future__ import annotations

import pathlib

from fret.config_loader import load_ros_parameters_yaml
from fret.scenario.dubins_race_runner import _mpc_config

_DUBINS_CTRL = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "controllers"
    / "dubins.yml"
)


def test_mpc_config_forwards_progress_first_fields_from_yaml() -> None:
    """``_mpc_config`` must round-trip weight_lag / contour_deadzone."""
    ctrl = load_ros_parameters_yaml(_DUBINS_CTRL)
    cfg = _mpc_config(ctrl)
    assert cfg.weight_lag == 12.0
    assert cfg.contour_deadzone == 0.10
    assert cfg.weight_contour == 4.0
    assert cfg.weight_heading == 3.0
    assert cfg.weight_progress == 3.0
    assert cfg.weight_terminal == 8.0
    assert cfg.horizon_step_count == 24
    assert cfg.dt == 0.05
    assert cfg.cruise_speed == float(ctrl["cruise_speed"])


def test_mpc_config_defaults_progress_first_when_keys_absent() -> None:
    """Missing lag/deadzone keys must not invent non-default values."""
    ctrl = {
        "cruise_speed": 0.36,
        "mpc": {
            "horizon_step_count": 10,
            "weight_contour": 1.0,
        },
    }
    cfg = _mpc_config(ctrl)
    assert cfg.horizon_step_count == 10
    assert cfg.weight_contour == 1.0
    assert cfg.weight_lag == 0.0
    assert cfg.contour_deadzone == 0.0
