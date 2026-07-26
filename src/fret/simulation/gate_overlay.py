"""Low-FPS gate-camera overlay clips for release showcase (v1.4+).

Side MP4s (``*_gate_cam_left.mp4`` / ``*_gate_cam_right.mp4``) sit next to the
overview encode in the same R2 folder. They are deliberately separate from the
overview compositor (no subwindows).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

GATE_CAMERAS: tuple[str, ...] = ("gate_cam_left", "gate_cam_right")
DEFAULT_OVERLAY_FPS = 5
# Match portal HSV tennis-yellow band (omx/omy_portal_overhead.yml).
_HSV_LOWER = (20, 40, 40)
_HSV_UPPER = (50, 255, 255)


def annotate_gate_frame(
    rgb: npt.NDArray[np.uint8],
    pose_xyz: Sequence[float] | None,
    *,
    label: str = "pose",
) -> npt.NDArray[np.uint8]:
    """Draw HSV blob circle + world-pose text on a MuJoCo RGB frame."""
    import cv2

    out = np.ascontiguousarray(rgb.copy())
    bgr = cv2.cvtColor(out, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(_HSV_LOWER, dtype=np.uint8),
        np.array(_HSV_UPPER, dtype=np.uint8),
    )
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) >= 40.0:
            (cx, cy), radius = cv2.minEnclosingCircle(largest)
            center = (int(round(cx)), int(round(cy)))
            cv2.circle(bgr, center, int(round(radius)), (0, 255, 255), 2)
            cv2.circle(bgr, center, 3, (0, 0, 255), -1)
    if pose_xyz is not None:
        x, y, z = (float(v) for v in pose_xyz[:3])
        lines = [
            f"{label}: x={x:+.3f} y={y:+.3f} z={z:+.3f}",
        ]
        y0 = 28
        for i, line in enumerate(lines):
            cv2.putText(
                bgr,
                line,
                (12, y0 + 28 * i),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (20, 20, 20),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                bgr,
                line,
                (12, y0 + 28 * i),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def present_gate_cameras(mujoco: Any, model: Any) -> list[str]:
    """Return gate camera names that exist on ``model``."""
    found: list[str] = []
    for name in GATE_CAMERAS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0:
            found.append(name)
    return found


def write_gate_cam_overlay_videos(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    samples: Sequence[Any],
    apply_sample: Callable[[Any], None],
    box_qadr: int,
    output_dir: Path,
    scenario: str,
    main_fps: int,
    overlay_fps: int = DEFAULT_OVERLAY_FPS,
    width: int = 640,
    height: int = 360,
    pose_label: str = "ball",
    cv_pose: npt.NDArray[np.float64] | None = None,
    open_writer: Callable[[Path, int], Any],
    frame_mean: Callable[[npt.NDArray[np.uint8]], float],
    output_name: Callable[[str, str], str],
    result_cls: type,
    timing: Any,
) -> list[Any]:
    """Encode low-FPS annotated gate-cam side clips; skip if cams missing."""
    cameras = present_gate_cameras(mujoco, model)
    if not cameras or not samples:
        return []

    stride = max(1, int(round(float(main_fps) / float(overlay_fps))))
    renderer = mujoco.Renderer(model, height=height, width=width)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        cam: output_dir / output_name(scenario, cam) for cam in cameras
    }
    writers = {cam: open_writer(paths[cam], overlay_fps) for cam in cameras}
    first_frames: dict[str, npt.NDArray[np.uint8]] = {}
    try:
        for i, sample in enumerate(samples):
            if i % stride != 0 and i != len(samples) - 1:
                continue
            apply_sample(sample)
            ball = np.asarray(
                data.qpos[box_qadr : box_qadr + 3], dtype=np.float64
            )
            pose = (
                np.asarray(cv_pose, dtype=np.float64).reshape(3)
                if cv_pose is not None
                else ball
            )
            label = "CV" if cv_pose is not None else pose_label
            for cam in cameras:
                renderer.update_scene(data, camera=cam)
                frame = annotate_gate_frame(
                    renderer.render(), pose, label=label
                )
                writers[cam].append_data(frame)
                if cam not in first_frames:
                    first_frames[cam] = frame.copy()
    finally:
        for writer in writers.values():
            writer.close()
        renderer.close()

    results: list[Any] = []
    for cam in cameras:
        mean = frame_mean(first_frames[cam])
        if mean <= 1.0:
            raise RuntimeError(
                f"Camera {cam!r} overlay looks blank (frame mean={mean:.2f})"
            )
        results.append(
            result_cls(
                camera=cam,
                path=paths[cam],
                frame_mean=mean,
                timing=timing,
            )
        )
    return results
