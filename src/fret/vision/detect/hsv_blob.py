"""HSV colour-blob ball detector (OpenCV)."""

from __future__ import annotations

from typing import Sequence

import numpy as np
import numpy.typing as npt

from fret.vision.config import HsvBlobDetectorConfig
from fret.vision.types import BallDetection, CameraFrame

try:
    import cv2
except ImportError as exc:  # pragma: no cover - exercised via importorskip
    cv2 = None  # type: ignore[assignment]
    _CV2_IMPORT_ERROR: BaseException | None = exc
else:
    _CV2_IMPORT_ERROR = None


def _require_cv2() -> None:
    if cv2 is None:
        raise ImportError(
            "opencv-python-headless is required for HsvBlobBallDetector"
        ) from _CV2_IMPORT_ERROR


def _to_bgr(image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
    """Convert RGB or gray frame to BGR for OpenCV."""
    _require_cv2()
    assert cv2 is not None
    if image.ndim == 2:
        out = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return np.asarray(out, dtype=np.uint8)
    if image.ndim == 3 and image.shape[2] == 3:
        out = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.asarray(out, dtype=np.uint8)
    raise ValueError(f"unsupported image shape {image.shape}")


def _circularity(area: float, perimeter: float) -> float:
    if perimeter <= 1e-9:
        return 0.0
    return float(4.0 * np.pi * area / (perimeter * perimeter))


class HsvBlobBallDetector:
    """Detect a ball via HSV threshold + contour circularity (single camera).

    Processes the first frame whose ``camera_id`` matches the config. Extra
    frames are ignored (MVP: one overhead view).
    """

    def __init__(self, config: HsvBlobDetectorConfig) -> None:
        _require_cv2()
        self._cfg = config

    @property
    def config(self) -> HsvBlobDetectorConfig:
        return self._cfg

    def mask_for_frame(self, frame: CameraFrame) -> npt.NDArray[np.uint8]:
        """Return the binary HSV+morphology mask for ``frame`` (H×W uint8)."""
        assert cv2 is not None
        bgr = _to_bgr(frame.image)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.asarray(self._cfg.hsv_lower, dtype=np.uint8)
        upper = np.asarray(self._cfg.hsv_upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        if (
            self._cfg.hsv_lower_alt is not None
            and self._cfg.hsv_upper_alt is not None
        ):
            lower_alt = np.asarray(self._cfg.hsv_lower_alt, dtype=np.uint8)
            upper_alt = np.asarray(self._cfg.hsv_upper_alt, dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_alt, upper_alt))
        k = int(self._cfg.morph_kernel_px)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return np.asarray(mask, dtype=np.uint8)

    def detect(self, frames: Sequence[CameraFrame]) -> list[BallDetection]:
        if not frames:
            return []
        frame = next(
            (f for f in frames if f.camera_id == self._cfg.camera_id), None
        )
        if frame is None:
            return []

        assert cv2 is not None
        mask = self.mask_for_frame(frame)
        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best: BallDetection | None = None
        best_score = -1.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._cfg.min_area_px or area > self._cfg.max_area_px:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            circ = _circularity(area, perimeter)
            if circ < self._cfg.min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            score = circ + 0.05 * min(
                1.0, area / max(self._cfg.max_area_px, 1.0)
            )
            if score > best_score:
                best_score = score
                best = BallDetection(
                    camera_id=frame.camera_id,
                    centre_px=(float(cx), float(cy)),
                    radius_px=float(radius),
                    confidence=float(np.clip(circ, 0.0, 1.0)),
                )
        return [best] if best is not None else []
