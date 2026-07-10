"""Physics SITL integration tests for PPP warehouse (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib

import pytest

from fret.config_loader import load_algorithm_config
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
_EE_ERROR_LIMIT_M = float(
    load_algorithm_config("planning/ppp.yml")["ee_error_limit_m"]
)


def _mujoco_available() -> bool:
    from fret.ros.mujoco_bridge import make_mujoco_bridge_core

    return make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_ppp_physics_run_completes_with_contact_log() -> None:
    """V12-5/7: physics_mode run produces contact log and zero obstacle penetration."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True, contact_log_enabled=True)
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.grasp_captured is True
    assert result.contact_log_path is not None
    assert result.contact_log_path.is_file()
    assert result.physics_metrics_path is not None
    assert result.physics_metrics_path.is_file()
    assert result.penetration_violations == 0


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
@pytest.mark.xfail(
    reason="Full PPP physics E2E tracking awaits v1.1.3 cargo weld (version_plan_v1.2.md)",
    strict=False,
)
def test_ppp_physics_tracking_within_limit_or_documented() -> None:
    """V12-2: physics tracking should approach the 10 mm EE limit (v1.1.4 target)."""
    runner = PPPWarehouseRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True)
    assert result.planning_status == PlanningStatus.SUCCESS
    assert result.max_tracking_error_m <= _EE_ERROR_LIMIT_M * 2.5
