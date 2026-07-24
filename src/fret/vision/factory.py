"""Factory helpers for the HSV + table-plane MVP pipeline."""

from __future__ import annotations

from pathlib import Path

from fret.vision.config import (
    HsvPlanePipelineConfig,
    load_hsv_plane_pipeline_config,
)
from fret.vision.detect.hsv_blob import HsvBlobBallDetector
from fret.vision.geometry.plane_lifter import TablePlanePoseLifter
from fret.vision.pipeline import BallVisionPipeline


def build_hsv_plane_pipeline(
    path: str | Path | None = None,
) -> BallVisionPipeline:
    """Build a wired :class:`BallVisionPipeline` from YAML (default overhead)."""
    cfg = load_hsv_plane_pipeline_config(path)
    return pipeline_from_hsv_plane_config(cfg)


def pipeline_from_hsv_plane_config(
    cfg: HsvPlanePipelineConfig,
) -> BallVisionPipeline:
    """Assemble detector + lifter from an already-loaded config bundle."""
    detector = HsvBlobBallDetector(cfg.detector)
    lifter = TablePlanePoseLifter(cfg.lifter, cfg.vision, source=cfg.source)
    return BallVisionPipeline(
        detector,
        lifter,
        config=cfg.vision,
        source=cfg.source,
    )
