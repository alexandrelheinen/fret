"""Load HSV blob + table-plane pipeline tunables from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import numpy.typing as npt

from fret.config_loader import load_yaml_file, require_key, require_keys
from fret.sitl_config import resolve_package_file
from fret.vision.types import (
    CameraExtrinsics,
    CameraIntrinsics,
    VisionConfig,
)


def _as_hsv_bound(
    values: Sequence[Any] | None, *, context: str
) -> tuple[int, int, int] | None:
    if values is None:
        return None
    if len(values) != 3:
        raise ValueError(f"{context} must have length 3, got {values!r}")
    return (int(values[0]), int(values[1]), int(values[2]))


def _require_hsv_bound(
    values: Sequence[Any], *, context: str
) -> tuple[int, int, int]:
    bound = _as_hsv_bound(values, context=context)
    assert bound is not None
    return bound


def _matrix4(
    raw: Sequence[Sequence[Any]], *, context: str
) -> npt.NDArray[np.float64]:
    matrix = np.asarray(raw, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"{context} must be 4x4, got shape {matrix.shape}")
    return matrix


@dataclass(frozen=True)
class HsvBlobDetectorConfig:
    """Tunables for :class:`~fret.vision.detect.hsv_blob.HsvBlobBallDetector`."""

    camera_id: str
    hsv_lower: tuple[int, int, int]
    hsv_upper: tuple[int, int, int]
    hsv_lower_alt: tuple[int, int, int] | None
    hsv_upper_alt: tuple[int, int, int] | None
    morph_kernel_px: int
    min_area_px: float
    max_area_px: float
    min_circularity: float

    def __post_init__(self) -> None:
        if self.morph_kernel_px < 1:
            raise ValueError("morph_kernel_px must be >= 1")
        if self.min_area_px < 0.0 or self.max_area_px < self.min_area_px:
            raise ValueError("invalid min_area_px / max_area_px")
        if not 0.0 <= self.min_circularity <= 1.0:
            raise ValueError("min_circularity must be in [0, 1]")
        if (self.hsv_lower_alt is None) != (self.hsv_upper_alt is None):
            raise ValueError(
                "hsv_lower_alt and hsv_upper_alt must both be set or both null"
            )


@dataclass(frozen=True)
class TablePlaneLifterConfig:
    """Tunables for table-plane pose lift."""

    camera_id: str
    table_z_m: float
    ball_radius_m: float

    def __post_init__(self) -> None:
        if self.ball_radius_m < 0.0:
            raise ValueError("ball_radius_m must be >= 0")


@dataclass(frozen=True)
class HsvPlanePipelineConfig:
    """Bundled YAML for the HSV + table-plane MVP pipeline."""

    vision: VisionConfig
    detector: HsvBlobDetectorConfig
    lifter: TablePlaneLifterConfig
    source: str


def _parse_intrinsics(raw: Mapping[str, Any]) -> CameraIntrinsics:
    require_keys(
        dict(raw),
        ("camera_id", "width", "height", "fx", "fy", "cx", "cy"),
        context="intrinsics",
    )
    return CameraIntrinsics(
        camera_id=str(raw["camera_id"]),
        width=int(raw["width"]),
        height=int(raw["height"]),
        fx=float(raw["fx"]),
        fy=float(raw["fy"]),
        cx=float(raw["cx"]),
        cy=float(raw["cy"]),
    )


def _parse_extrinsics(raw: Mapping[str, Any]) -> CameraExtrinsics:
    require_keys(dict(raw), ("camera_id", "t_world_cam"), context="extrinsics")
    return CameraExtrinsics(
        camera_id=str(raw["camera_id"]),
        t_world_cam=_matrix4(
            raw["t_world_cam"], context="extrinsics.t_world_cam"
        ),
    )


def load_hsv_plane_pipeline_config(
    path: str | Path | None = None,
) -> HsvPlanePipelineConfig:
    """Load ``hsv_blob_overhead.yml`` (or ``path``) into typed configs."""
    if path is None:
        file_path = resolve_package_file(
            "config", "vision", "hsv_blob_overhead.yml"
        )
    else:
        file_path = Path(path)
    data = load_yaml_file(file_path)
    camera_id = str(require_key(data, "camera_id", context=str(file_path)))

    intr = _parse_intrinsics(
        require_key(data, "intrinsics", context=str(file_path))
    )
    ext = _parse_extrinsics(
        require_key(data, "extrinsics", context=str(file_path))
    )
    if intr.camera_id != camera_id or ext.camera_id != camera_id:
        raise ValueError(
            f"camera_id mismatch in {file_path}: "
            f"top={camera_id!r} intr={intr.camera_id!r} ext={ext.camera_id!r}"
        )

    det_raw = require_key(data, "detector", context=str(file_path))
    if not isinstance(det_raw, dict):
        raise ValueError("detector must be a mapping")
    require_keys(
        det_raw,
        (
            "hsv_lower",
            "hsv_upper",
            "morph_kernel_px",
            "min_area_px",
            "max_area_px",
            "min_circularity",
        ),
        context="detector",
    )
    detector = HsvBlobDetectorConfig(
        camera_id=camera_id,
        hsv_lower=_require_hsv_bound(
            det_raw["hsv_lower"], context="hsv_lower"
        ),
        hsv_upper=_require_hsv_bound(
            det_raw["hsv_upper"], context="hsv_upper"
        ),
        hsv_lower_alt=_as_hsv_bound(
            det_raw.get("hsv_lower_alt"), context="hsv_lower_alt"
        ),
        hsv_upper_alt=_as_hsv_bound(
            det_raw.get("hsv_upper_alt"), context="hsv_upper_alt"
        ),
        morph_kernel_px=int(det_raw["morph_kernel_px"]),
        min_area_px=float(det_raw["min_area_px"]),
        max_area_px=float(det_raw["max_area_px"]),
        min_circularity=float(det_raw["min_circularity"]),
    )

    lift_raw = require_key(data, "lifter", context=str(file_path))
    if not isinstance(lift_raw, dict):
        raise ValueError("lifter must be a mapping")
    require_keys(lift_raw, ("table_z_m", "ball_radius_m"), context="lifter")
    lifter = TablePlaneLifterConfig(
        camera_id=camera_id,
        table_z_m=float(lift_raw["table_z_m"]),
        ball_radius_m=float(lift_raw["ball_radius_m"]),
    )

    pipe_raw = require_key(data, "pipeline", context=str(file_path))
    if not isinstance(pipe_raw, dict):
        raise ValueError("pipeline must be a mapping")
    source = str(require_key(pipe_raw, "source", context="pipeline"))
    if not source:
        raise ValueError("pipeline.source must be non-empty")

    return HsvPlanePipelineConfig(
        vision=VisionConfig(intrinsics=(intr,), extrinsics=(ext,)),
        detector=detector,
        lifter=lifter,
        source=source,
    )
