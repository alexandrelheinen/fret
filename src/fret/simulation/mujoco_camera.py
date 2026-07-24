"""MuJoCo named-camera → ``fret.vision`` adapter (v1.4 T14-01/T14-02).

Converts MuJoCo camera poses to OpenCV optical frames, derives pinhole
intrinsics from ``fovy``, and renders RGB into :class:`CameraFrame`.

Does **not** feed detections into the pick-and-place FSM — that is T14-03.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.vision.types import (
    CameraExtrinsics,
    CameraFrame,
    CameraIntrinsics,
)

# Perception render size for SC-v16 camera YAML + CI benchmarks.
PERCEPTION_WIDTH_PX = 1280
PERCEPTION_HEIGHT_PX = 720
PERCEPTION_CAMERA_ID = "overhead"


def intrinsics_from_fovy(
    *,
    camera_id: str,
    width: int,
    height: int,
    fovy_deg: float,
) -> CameraIntrinsics:
    """Pinhole intrinsics matching MuJoCo's vertical field-of-view model."""
    if width <= 0 or height <= 0:
        raise ValueError(
            f"width/height must be positive, got {width}x{height}"
        )
    if fovy_deg <= 0.0 or fovy_deg >= 180.0:
        raise ValueError(f"fovy_deg must be in (0, 180), got {fovy_deg}")
    fy = 0.5 * float(height) / float(np.tan(np.deg2rad(fovy_deg) * 0.5))
    fx = fy
    cx = 0.5 * float(width - 1)
    cy = 0.5 * float(height - 1)
    return CameraIntrinsics(
        camera_id=camera_id,
        width=width,
        height=height,
        fx=float(fx),
        fy=float(fy),
        cx=cx,
        cy=cy,
    )


def mujoco_xmat_to_opencv_rotation(
    xmat: npt.NDArray[np.floating[Any]],
) -> npt.NDArray[np.float64]:
    """Map MuJoCo camera axes to OpenCV optical axes in the world frame.

    MuJoCo cameras look along ``-Z`` with ``+Y`` up in the OpenGL image.
    OpenCV optical: ``+Z`` forward (into the scene), ``+Y`` down, ``+X`` right.
    """
    rotation_mj = np.asarray(xmat, dtype=np.float64).reshape(3, 3)
    return rotation_mj @ np.diag([1.0, -1.0, -1.0])


def extrinsics_from_mujoco_camera(
    *,
    camera_id: str,
    xpos: npt.NDArray[np.floating[Any]],
    xmat: npt.NDArray[np.floating[Any]],
) -> CameraExtrinsics:
    """Build ``T_world_cam`` (OpenCV) from MuJoCo ``cam_xpos`` / ``cam_xmat``."""
    eye = np.asarray(xpos, dtype=np.float64).reshape(3)
    rotation = mujoco_xmat_to_opencv_rotation(xmat)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = eye
    return CameraExtrinsics(camera_id=camera_id, t_world_cam=matrix)


def project_world_point(
    point_world: npt.NDArray[np.floating[Any]],
    intrinsics: CameraIntrinsics,
    extrinsics: CameraExtrinsics,
) -> tuple[float, float]:
    """Project a world point to pixel ``(u, v)`` with the OpenCV pinhole model."""
    point = np.asarray(point_world, dtype=np.float64).reshape(3)
    rotation = extrinsics.t_world_cam[:3, :3]
    origin = extrinsics.t_world_cam[:3, 3]
    point_cam = rotation.T @ (point - origin)
    z = float(point_cam[2])
    if z <= 1e-9:
        raise ValueError(f"point is behind / on the camera (z={z})")
    u = intrinsics.fx * float(point_cam[0]) / z + intrinsics.cx
    v = intrinsics.fy * float(point_cam[1]) / z + intrinsics.cy
    return float(u), float(v)


@dataclass(frozen=True)
class MujocoCameraCapture:
    """One RGB frame plus calibration derived from the live MuJoCo model."""

    frame: CameraFrame
    intrinsics: CameraIntrinsics
    extrinsics: CameraExtrinsics


class MujocoCameraAdapter:
    """Render a named MuJoCo camera into a :class:`CameraFrame`.

    Calibration is read from the model/data each capture so MJCF edits stay
    authoritative; YAML configs should match within the unit-test tolerance.
    """

    def __init__(
        self,
        model: Any,
        data: Any,
        *,
        camera_name: str = PERCEPTION_CAMERA_ID,
        width: int = PERCEPTION_WIDTH_PX,
        height: int = PERCEPTION_HEIGHT_PX,
        mujoco_module: Any | None = None,
    ) -> None:
        if mujoco_module is None:
            import mujoco as mj_mod
        else:
            mj_mod = mujoco_module
        self._mj: Any = mj_mod
        self._model = model
        self._data = data
        self._camera_name = camera_name
        self._width = int(width)
        self._height = int(height)
        cam_id = self._mj.mj_name2id(
            model, self._mj.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if cam_id < 0:
            raise ValueError(f"Camera not found in MJCF: {camera_name}")
        self._cam_id = int(cam_id)
        self._renderer = self._mj.Renderer(model, height=height, width=width)

    @property
    def camera_name(self) -> str:
        return self._camera_name

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    def close(self) -> None:
        close = getattr(self._renderer, "close", None)
        if callable(close):
            close()

    def __enter__(self) -> MujocoCameraAdapter:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def live_intrinsics(self) -> CameraIntrinsics:
        fovy = float(self._model.cam_fovy[self._cam_id])
        return intrinsics_from_fovy(
            camera_id=self._camera_name,
            width=self._width,
            height=self._height,
            fovy_deg=fovy,
        )

    def live_extrinsics(self) -> CameraExtrinsics:
        self._mj.mj_forward(self._model, self._data)
        return extrinsics_from_mujoco_camera(
            camera_id=self._camera_name,
            xpos=self._data.cam_xpos[self._cam_id],
            xmat=self._data.cam_xmat[self._cam_id],
        )

    def capture(self, timestamp: float = 0.0) -> MujocoCameraCapture:
        self._mj.mj_forward(self._model, self._data)
        intrinsics = self.live_intrinsics()
        extrinsics = self.live_extrinsics()
        self._renderer.update_scene(self._data, camera=self._camera_name)
        image = np.asarray(self._renderer.render(), dtype=np.uint8)
        frame = CameraFrame(
            camera_id=self._camera_name,
            image=image,
            timestamp=float(timestamp),
            intrinsics_id=self._camera_name,
        )
        return MujocoCameraCapture(
            frame=frame, intrinsics=intrinsics, extrinsics=extrinsics
        )
