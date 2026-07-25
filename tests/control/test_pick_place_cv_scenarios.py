"""SC-v16a/b — CV pick-place scenario registration + physics smoke (T14-04)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_OMX_CV = Path("src/fret/config/scenarios/omx_pick_place_cv.yml")
_OMY_CV = Path("src/fret/config/scenarios/omy_pick_place_cv.yml")


@pytest.mark.parametrize(
    ("path", "model", "scenario_id", "showcase_id", "vision_cfg"),
    [
        (
            _OMX_CV,
            "open_manipulator_x",
            "omx_pick_place",
            "omx_pick_place_cv",
            "src/fret/config/vision/omx_portal_overhead.yml",
        ),
        (
            _OMY_CV,
            "omy",
            "omy_pick_place",
            "omy_pick_place_cv",
            "src/fret/config/vision/omy_portal_overhead.yml",
        ),
    ],
)
def test_sc_v16_cv_scenario_registration(
    path: Path,
    model: str,
    scenario_id: str,
    showcase_id: str,
    vision_cfg: str,
) -> None:
    params = load_scenario_parameters(path)
    assert params["model"] == model
    assert params["scenario_id"] == scenario_id
    assert params["showcase_id"] == showcase_id
    assert params.get("use_vision") is True
    assert params["vision_config"] == vision_cfg
    assert "place_xy" in params
    assert "pick_xy" in params  # oracle only
    xml = mjcf_path(model, scenario_id)
    assert xml.is_file()
    assert "pick_place" in xml.name


def test_omx_pick_place_cv_physics_done() -> None:
    from fret.control.pick_place_fsm import PickPlaceState
    from fret.control.pick_place_sim import run_pick_place

    params = load_scenario_parameters(_OMX_CV)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    state, ball = run_pick_place(
        duration_s=25.0,
        scenario_path=_OMX_CV,
        use_vision=True,
    )
    assert state != PickPlaceState.FAULT
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    assert float(np.linalg.norm(ball[:2] - place_xy)) < 0.08
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    assert float(ball[2]) < 0.08


@pytest.mark.slow
def test_omy_pick_place_cv_physics_done() -> None:
    from fret.control.omy_pick_place_sim import run_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    params = load_scenario_parameters(_OMY_CV)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    cone_r = float(params.get("place_cone_radius_m", 0.14))
    state, ball = run_omy_pick_place(
        duration_s=45.0,
        joint_tol_rad=0.16,
        scenario_path=_OMY_CV,
        use_vision=True,
    )
    assert state != PickPlaceState.FAULT
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    assert float(np.linalg.norm(ball[:2] - place_xy)) < cone_r
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    assert float(ball[2]) < float(params.get("place_cone_height_m", 0.28))
