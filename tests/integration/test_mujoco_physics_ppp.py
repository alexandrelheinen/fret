"""Physics SITL integration tests for PPP warehouse (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib

import pytest

from fret.config_loader import load_scenario_bundle
from fret.interfaces import PlanningStatus
from fret.scenario.ppp_warehouse_runner import PPPWarehouseRunner

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "ppp_warehouse.yml"
)


def _mujoco_available() -> bool:
    from fret.ros.mujoco_bridge import make_mujoco_bridge_core

    return make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime


def _physics_tracking_limit_m() -> float:
    planning = load_scenario_bundle(_SCENARIO_PATH).planning
    kinematic = float(planning["ee_error_limit_m"])
    ratio = float(planning.get("dubins_physics_rtf_ratio", 1.73))
    return float(
        planning.get(
            "ee_error_limit_physics_m",
            kinematic * ratio,
        )
    )


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_ppp_physics_run_completes_with_contact_log() -> None:
    """V12-5/7: physics_mode run produces contact log and zero obstacle penetration."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True, contact_log_enabled=True)
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.grasp_captured is True
    assert result.grasp_released is True
    assert result.contact_log_path is not None
    assert result.contact_log_path.is_file()
    assert result.physics_metrics_path is not None
    assert result.physics_metrics_path.is_file()
    assert result.penetration_violations == 0


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_ppp_physics_tracking_within_v12_limit() -> None:
    """V12-2: physics E2E tracking within relaxed gate (kinematic × Dubins RTF)."""
    limit_m = _physics_tracking_limit_m()
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True)
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.grasp_captured is True
    assert result.grasp_released is True
    assert result.controller_faulted is False
    assert result.max_tracking_error_m <= limit_m
