"""Vision → pick-goal wiring for pick-and-place (v1.4 T14-03)."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("cv2")

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import mujoco

from fret.control.omy_pick_place_sim import waypoints_from_scenario as omy_wp
from fret.control.pick_place_sim import waypoints_from_scenario as omx_wp
from fret.control.pick_place_vision import (
    apply_vision_pick_goals,
    observe_ball_mujoco,
    refresh_pick_waypoints_omx,
)
from fret.mjcf.omx import ensure_omx_mjcf
from fret.mjcf.omy import ensure_omy_mjcf
from fret.sitl_config import load_scenario_parameters

_OMX_SCENARIO = Path("src/fret/config/scenarios/omx_pick_place.yml")
_OMY_SCENARIO = Path("src/fret/config/scenarios/omy_pick_place.yml")


def _load(xml: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_observe_ball_omx_gate_cameras() -> None:
    model, data = _load(ensure_omx_mjcf("omx_pick_place"))
    obs = observe_ball_mujoco(model, data, robot="omx")
    assert obs is not None
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    true = np.asarray(data.xpos[ball_id], dtype=np.float64)
    err = float(np.linalg.norm(obs.position_world[:2] - true[:2]))
    assert err < 0.015, f"vision XY error {err * 1000:.1f} mm"


def test_apply_vision_pick_ignores_yaml_pick_xy_omx() -> None:
    """Pick joints come from BallObservation, not scenario pick_xy."""
    params = load_scenario_parameters(_OMX_SCENARIO)
    yaml_pick = np.asarray(params["pick_xy"], dtype=np.float64)
    model, data = _load(ensure_omx_mjcf("omx_pick_place"))
    base = omx_wp()
    # Poison YAML-shaped pick joints so a regression that reuses them fails.
    poisoned = base.with_pick(
        pick_hover=base.idle.copy(),
        pick_grasp=base.idle.copy(),
        lift_hover=base.idle.copy(),
    )
    refreshed, ball_obs = apply_vision_pick_goals(
        poisoned, robot="omx", model=model, data=data
    )
    assert not np.allclose(refreshed.pick_grasp, poisoned.pick_grasp)
    assert np.allclose(refreshed.place_hover, base.place_hover)
    # Vision ball near MJCF ball, not necessarily equal to YAML pick_xy
    # (YAML remains an oracle; both should agree for the default cell).
    assert (
        float(np.linalg.norm(ball_obs.position_world[:2] - yaml_pick)) < 0.05
    )


def test_refresh_pick_waypoints_omx_shifts_with_ball() -> None:
    base = omx_wp()
    a = refresh_pick_waypoints_omx(base, np.array([0.22, -0.20, 0.11]))
    b = refresh_pick_waypoints_omx(base, np.array([0.22, 0.20, 0.11]))
    assert not np.allclose(a.pick_grasp, b.pick_grasp)
    assert np.allclose(a.place_grasp, b.place_grasp)


def test_apply_vision_pick_omy() -> None:
    model, data = _load(ensure_omy_mjcf("omy_pick_place"))
    base = omy_wp(_OMY_SCENARIO)
    refreshed, obs = apply_vision_pick_goals(
        base, robot="omy", model=model, data=data
    )
    assert obs is not None
    assert not np.allclose(refreshed.pick_grasp, base.idle)
    assert np.allclose(refreshed.place_hover, base.place_hover)
