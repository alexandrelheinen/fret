"""MuJoCo rear-gate dual-camera + HSV fusion validation (v1.4).

Adapter / detect / lift gates only. FSM wiring: ``test_pick_place_vision``.
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
    PERCEPTION_HEIGHT_PX,
    PERCEPTION_WIDTH_PX,
    MujocoCameraAdapter,
    project_world_point,
)
from fret.vision.config import load_hsv_plane_pipeline_config
from fret.vision.detect.hsv_blob import HsvBlobBallDetector
from fret.vision.factory import build_hsv_plane_pipeline
from fret.vision.types import VisionConfig

_MAX_CENTRE_ERR_PX = 5.0
_MAX_XY_ERR_OMX_M = 0.015
_MAX_XY_ERR_OMY_M = 0.020
_MAX_Z_ERR_M = 0.005
_CALIB_POS_TOL_M = 1e-5
_CALIB_ROT_TOL = 1e-5

_OMX_CFG = Path("src/fret/config/vision/omx_portal_overhead.yml")
_OMY_CFG = Path("src/fret/config/vision/omy_portal_overhead.yml")
_CAMS = ("gate_cam_left", "gate_cam_right")


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
    cfg_path: Path, model: mujoco.MjModel, data: mujoco.MjData
) -> None:
    cfg = load_hsv_plane_pipeline_config(cfg_path)
    for name in _CAMS:
        with MujocoCameraAdapter(model, data, camera_name=name) as adapter:
            yaml_intr = cfg.vision.intrinsics_for(name)
            live_intr = adapter.live_intrinsics()
            assert yaml_intr.width == live_intr.width == PERCEPTION_WIDTH_PX
            assert yaml_intr.height == live_intr.height == PERCEPTION_HEIGHT_PX
            assert yaml_intr.fx == pytest.approx(live_intr.fx, rel=0, abs=1e-6)
            yaml_ext = cfg.vision.extrinsics_for(name)
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


def _run_gate_benchmark(
    *,
    scene_xml: Path,
    cfg_path: Path,
    max_xy_err_m: float,
) -> dict[str, float]:
    model, data = _load_scene(scene_xml)
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "vision_gate") >= 0
    )
    for name in _CAMS:
        assert (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0
        ), name

    pipeline = build_hsv_plane_pipeline(cfg_path)
    detector = HsvBlobBallDetector(
        load_hsv_plane_pipeline_config(cfg_path).detector
    )
    _assert_yaml_matches_live(cfg_path, model, data)

    frames = []
    captures = []
    try:
        adapters = [
            MujocoCameraAdapter(model, data, camera_name=name)
            for name in _CAMS
        ]
        for adapter in adapters:
            try:
                cap = adapter.capture(timestamp=0.0)
            except RuntimeError as exc:
                pytest.skip(f"MuJoCo offscreen render unavailable: {exc}")
            frames.append(cap.frame)
            captures.append(cap)
    finally:
        for adapter in adapters:
            adapter.close()

    ball = _ball_world(model, data)
    detections = detector.detect(frames)
    assert detections, "HSV detector missed ball on both gate cameras"
    # Per-view centre error vs projection (only for cameras that hit).
    centre_errors = []
    for det in detections:
        cap = next(c for c in captures if c.frame.camera_id == det.camera_id)
        u_gt, v_gt = project_world_point(ball, cap.intrinsics, cap.extrinsics)
        centre_errors.append(
            float(np.hypot(det.centre_px[0] - u_gt, det.centre_px[1] - v_gt))
        )
    centre_err = float(max(centre_errors))
    assert (
        centre_err <= _MAX_CENTRE_ERR_PX
    ), f"image centre error {centre_err:.2f} px > {_MAX_CENTRE_ERR_PX}"

    observation = pipeline.process(frames)
    assert observation is not None, "fused pose lifter returned None"
    xy_err = float(np.linalg.norm(observation.position_world[:2] - ball[:2]))
    z_err = float(abs(observation.position_world[2] - ball[2]))
    assert (
        xy_err <= max_xy_err_m
    ), f"world XY error {xy_err*1000:.1f} mm > {max_xy_err_m*1000:.0f} mm"
    assert (
        z_err <= _MAX_Z_ERR_M
    ), f"world Z error {z_err*1000:.1f} mm > {_MAX_Z_ERR_M*1000:.0f} mm"

    # One-camera obstruction: drop the higher-confidence view if both present.
    if len(detections) >= 2:
        kept = min(detections, key=lambda d: d.confidence)
        frames_one = [f for f in frames if f.camera_id == kept.camera_id]
        obs_one = pipeline.process(frames_one)
        assert obs_one is not None
        xy_one = float(
            np.linalg.norm(obs_one.position_world[:2] - ball[:2])
        )
        assert xy_one <= max_xy_err_m * 1.5

    return {
        "centre_err_px": centre_err,
        "xy_err_m": xy_err,
        "z_err_m": z_err,
        "n_detections": float(len(detections)),
    }


@pytest.mark.parametrize(
    ("ensure", "scene", "cfg", "max_xy"),
    [
        (ensure_omx_mjcf, "omx_pick_place", _OMX_CFG, _MAX_XY_ERR_OMX_M),
        (ensure_omy_mjcf, "omy_pick_place", _OMY_CFG, _MAX_XY_ERR_OMY_M),
    ],
)
def test_gate_dual_camera_vision_benchmark(
    ensure: object,
    scene: str,
    cfg: Path,
    max_xy: float,
) -> None:
    xml = ensure(scene)  # type: ignore[operator]
    metrics = _run_gate_benchmark(
        scene_xml=xml, cfg_path=cfg, max_xy_err_m=max_xy
    )
    assert metrics["n_detections"] >= 1.0


@pytest.mark.parametrize(
    ("ensure", "scene"),
    [
        (ensure_omx_mjcf, "omx_desk_clutter"),
        (ensure_omx_mjcf, "omx_wall_maze"),
        (ensure_omy_mjcf, "omy_clutter"),
    ],
)
def test_gate_present_on_sibling_cells(ensure: object, scene: str) -> None:
    xml = ensure(scene)  # type: ignore[operator]
    model, _data = _load_scene(xml)
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "vision_gate") >= 0
    )
    for name in _CAMS:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0


def test_perception_resolution_and_dual_yaml() -> None:
    assert PERCEPTION_WIDTH_PX == 1280
    assert PERCEPTION_HEIGHT_PX == 720
    for path in (_OMX_CFG, _OMY_CFG):
        cfg = load_hsv_plane_pipeline_config(path)
        assert isinstance(cfg.vision, VisionConfig)
        assert cfg.detector.camera_ids == _CAMS
        assert cfg.lifter.camera_ids == _CAMS
        for name in _CAMS:
            assert cfg.vision.intrinsics_for(name).width == 1280
