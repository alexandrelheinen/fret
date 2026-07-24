"""MuJoCo portal camera + HSV pipeline validation (v1.4 T14-01/T14-02).

Stops before wiring ``BallObservation`` into the pick-and-place FSM (T14-03).
"""

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

from fret.mjcf.omx import ensure_omx_mjcf
from fret.mjcf.omy import ensure_omy_mjcf
from fret.simulation.mujoco_camera import (
    PERCEPTION_CAMERA_ID,
    PERCEPTION_HEIGHT_PX,
    PERCEPTION_WIDTH_PX,
    MujocoCameraAdapter,
    project_world_point,
)
from fret.vision.config import load_hsv_plane_pipeline_config
from fret.vision.detect.hsv_blob import HsvBlobBallDetector
from fret.vision.factory import build_hsv_plane_pipeline
from fret.vision.types import VisionConfig

# Benchmark gates (docs/vision/camera-layout.md).
_MAX_CENTRE_ERR_PX = 5.0
_MAX_XY_ERR_OMX_M = 0.015
_MAX_XY_ERR_OMY_M = 0.020
_MAX_Z_ERR_M = 0.005
_CALIB_POS_TOL_M = 1e-6
_CALIB_ROT_TOL = 1e-6

_OMX_CFG = Path("src/fret/config/vision/omx_portal_overhead.yml")
_OMY_CFG = Path("src/fret/config/vision/omy_portal_overhead.yml")


def _load_scene(xml_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _ball_world(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    assert body_id >= 0
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy()


def _assert_yaml_matches_live(
    cfg_path: Path, adapter: MujocoCameraAdapter
) -> None:
    cfg = load_hsv_plane_pipeline_config(cfg_path)
    yaml_intr = cfg.vision.intrinsics_for(PERCEPTION_CAMERA_ID)
    live_intr = adapter.live_intrinsics()
    assert yaml_intr.width == live_intr.width == PERCEPTION_WIDTH_PX
    assert yaml_intr.height == live_intr.height == PERCEPTION_HEIGHT_PX
    assert yaml_intr.fx == pytest.approx(live_intr.fx, rel=0, abs=1e-6)
    assert yaml_intr.fy == pytest.approx(live_intr.fy, rel=0, abs=1e-6)
    assert yaml_intr.cx == pytest.approx(live_intr.cx, rel=0, abs=1e-9)
    assert yaml_intr.cy == pytest.approx(live_intr.cy, rel=0, abs=1e-9)

    yaml_ext = cfg.vision.extrinsics_for(PERCEPTION_CAMERA_ID)
    live_ext = adapter.live_extrinsics()
    assert np.allclose(
        yaml_ext.t_world_cam[:3, 3],
        live_ext.t_world_cam[:3, 3],
        atol=_CALIB_POS_TOL_M,
    )
    assert np.allclose(
        yaml_ext.t_world_cam[:3, :3],
        live_ext.t_world_cam[:3, :3],
        atol=_CALIB_ROT_TOL,
    )


def _run_portal_benchmark(
    *,
    scene_xml: Path,
    cfg_path: Path,
    max_xy_err_m: float,
) -> dict[str, float]:
    model, data = _load_scene(scene_xml)
    cam_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, PERCEPTION_CAMERA_ID
    )
    assert cam_id >= 0, "overhead camera missing from MJCF"
    portal_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "vision_portal"
    )
    assert portal_id >= 0, "vision_portal body missing from MJCF"

    pipeline = build_hsv_plane_pipeline(cfg_path)
    detector = HsvBlobBallDetector(
        load_hsv_plane_pipeline_config(cfg_path).detector
    )

    with MujocoCameraAdapter(model, data) as adapter:
        _assert_yaml_matches_live(cfg_path, adapter)
        try:
            capture = adapter.capture(timestamp=0.0)
        except RuntimeError as exc:
            pytest.skip(f"MuJoCo offscreen render unavailable: {exc}")
        ball = _ball_world(model, data)
        u_gt, v_gt = project_world_point(
            ball, capture.intrinsics, capture.extrinsics
        )

        detections = detector.detect([capture.frame])
        assert len(detections) == 1, "HSV detector missed MuJoCo ball"
        u_det, v_det = detections[0].centre_px
        centre_err = float(np.hypot(u_det - u_gt, v_det - v_gt))
        assert (
            centre_err <= _MAX_CENTRE_ERR_PX
        ), f"image centre error {centre_err:.2f} px > {_MAX_CENTRE_ERR_PX}"

        # Pipeline uses YAML calibration (must match live within tol above).
        observation = pipeline.process([capture.frame])
        assert observation is not None, "pose lifter returned None"
        xy_err = float(
            np.linalg.norm(observation.position_world[:2] - ball[:2])
        )
        z_err = float(abs(observation.position_world[2] - ball[2]))
        assert (
            xy_err <= max_xy_err_m
        ), f"world XY error {xy_err*1000:.1f} mm > {max_xy_err_m*1000:.0f} mm"
        assert (
            z_err <= _MAX_Z_ERR_M
        ), f"world Z error {z_err*1000:.1f} mm > {_MAX_Z_ERR_M*1000:.0f} mm"

    return {
        "centre_err_px": centre_err,
        "xy_err_m": xy_err,
        "z_err_m": z_err,
        "u_gt": u_gt,
        "v_gt": v_gt,
        "u_det": float(u_det),
        "v_det": float(v_det),
    }


@pytest.mark.parametrize(
    ("ensure", "scene", "cfg", "max_xy"),
    [
        (ensure_omx_mjcf, "omx_pick_place", _OMX_CFG, _MAX_XY_ERR_OMX_M),
        (ensure_omy_mjcf, "omy_pick_place", _OMY_CFG, _MAX_XY_ERR_OMY_M),
    ],
)
def test_portal_vision_benchmark(
    ensure: object,
    scene: str,
    cfg: Path,
    max_xy: float,
) -> None:
    xml = ensure(scene)  # type: ignore[operator]
    metrics = _run_portal_benchmark(
        scene_xml=xml, cfg_path=cfg, max_xy_err_m=max_xy
    )
    assert metrics["centre_err_px"] >= 0.0


@pytest.mark.parametrize(
    ("ensure", "scene"),
    [
        (ensure_omx_mjcf, "omx_desk_clutter"),
        (ensure_omx_mjcf, "omx_wall_maze"),
        (ensure_omy_mjcf, "omy_clutter"),
    ],
)
def test_portal_camera_present_on_clutter_cells(
    ensure: object, scene: str
) -> None:
    xml = ensure(scene)  # type: ignore[operator]
    model, _data = _load_scene(xml)
    assert (
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, PERCEPTION_CAMERA_ID
        )
        >= 0
    )
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "vision_portal")
        >= 0
    )


def test_perception_resolution_constants() -> None:
    assert PERCEPTION_WIDTH_PX == 1280
    assert PERCEPTION_HEIGHT_PX == 720
    for path in (_OMX_CFG, _OMY_CFG):
        cfg = load_hsv_plane_pipeline_config(path)
        intr = cfg.vision.intrinsics_for("overhead")
        assert intr.width == 1280
        assert intr.height == 720


def test_vision_config_accepts_portal_yaml() -> None:
    cfg = load_hsv_plane_pipeline_config(_OMX_CFG)
    assert isinstance(cfg.vision, VisionConfig)
    assert cfg.lifter.ball_radius_m == pytest.approx(0.0125)
