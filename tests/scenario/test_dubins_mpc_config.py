"""Unit tests for Dubins race MPC config wiring (classical MPCC fields)."""

from __future__ import annotations

import pathlib

import pytest
from arco.control.mpc import PathFollowingMPCConfig

from fret.config_loader import load_ros_parameters_yaml
from fret.scenario.dubins_race_runner import (
    _build_vehicle_mpc_sim,
    _mpc_config,
    _vehicle_config,
)

_DUBINS_CTRL = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "controllers"
    / "dubins.yml"
)


def test_mpc_config_forwards_mpcc_fields_from_yaml() -> None:
    """``_mpc_config`` must round-trip classical MPCC weights from YAML."""
    ctrl = load_ros_parameters_yaml(_DUBINS_CTRL)
    cfg = _mpc_config(ctrl)
    assert cfg.weight_lag == 6.0
    assert cfg.contour_deadzone == 0.0
    assert cfg.weight_contour == 10.0
    assert cfg.weight_heading == 1.0
    assert cfg.weight_progress == 4.0
    assert cfg.weight_terminal == 12.0
    assert cfg.horizon_step_count == 24
    assert cfg.dt == 0.05
    assert cfg.cruise_speed == float(ctrl["cruise_speed"])
    assert not hasattr(cfg, "weight_slack")


def test_mpc_config_defaults_lag_positive_when_keys_absent() -> None:
    """Missing lag/deadzone keys keep ARCO defaults; lag must stay > 0."""
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
    assert cfg.weight_lag > 0.0
    assert cfg.contour_deadzone == 0.0


def test_zero_lag_rejected_by_arco_mpcc() -> None:
    """ARCO ≥ v0.3.7 rejects lag ≤ 0 at tracker construction."""
    from arco.control.mpc import DubinsPathFollowingMPC, DubinsVehicleLimits

    with pytest.raises(ValueError, match="weight_lag"):
        DubinsPathFollowingMPC(
            vehicle_limits=DubinsVehicleLimits(
                max_speed=1.0,
                min_speed=0.0,
                max_turn_rate=1.0,
                max_acceleration=1.0,
                max_turn_rate_dot=1.0,
            ),
            config=PathFollowingMPCConfig(weight_lag=0.0),
        )


def test_build_vehicle_mpc_sim_preserves_mpcc_fields() -> None:
    """Live tracker must keep lag/deadzone into the NLP config."""
    ctrl = load_ros_parameters_yaml(_DUBINS_CTRL)
    vehicle_cfg = _vehicle_config(ctrl)
    mpc_cfg = _mpc_config(ctrl)
    _vehicle, loop = _build_vehicle_mpc_sim(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        vehicle_cfg,
        mpc_cfg,
        occupancy=None,
    )
    live = loop.tracker.config
    assert live.weight_lag == mpc_cfg.weight_lag == 6.0
    assert live.contour_deadzone == mpc_cfg.contour_deadzone == 0.0
    assert live.weight_contour == mpc_cfg.weight_contour
    assert live.cruise_speed == vehicle_cfg.cruise_speed
