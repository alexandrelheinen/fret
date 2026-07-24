#!/usr/bin/env python3
"""Benchmark HSV portal vision on MuJoCo OM-X / OMY pick-place cells.

Renders the portal ``overhead`` camera, runs ``fret.vision``, and reports
image-centre error vs geometric projection plus world-frame pose error vs
``pick_box`` ground truth. Does **not** drive the pick-and-place FSM.

Example::

    MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \\
      python3 scripts/benchmark_mujoco_vision.py \\
      --out-dir /tmp/fret_vision_bench
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))


def _run_one(
    *,
    label: str,
    ensure_fn: object,
    scene: str,
    cfg_path: Path,
) -> dict[str, float | str]:
    import mujoco

    from fret.simulation.mujoco_camera import (
        MujocoCameraAdapter,
        project_world_point,
    )
    from fret.vision.config import load_hsv_plane_pipeline_config
    from fret.vision.detect.hsv_blob import HsvBlobBallDetector
    from fret.vision.factory import build_hsv_plane_pipeline

    xml = ensure_fn(scene)  # type: ignore[operator]
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    ball = np.asarray(data.xpos[body_id], dtype=np.float64).copy()

    cfg = load_hsv_plane_pipeline_config(cfg_path)
    pipeline = build_hsv_plane_pipeline(cfg_path)
    detector = HsvBlobBallDetector(cfg.detector)

    with MujocoCameraAdapter(model, data) as adapter:
        capture = adapter.capture()
        u_gt, v_gt = project_world_point(
            ball, capture.intrinsics, capture.extrinsics
        )
        detections = detector.detect([capture.frame])
        if not detections:
            raise RuntimeError(f"{label}: no detection")
        u_det, v_det = detections[0].centre_px
        obs = pipeline.process([capture.frame])
        if obs is None:
            raise RuntimeError(f"{label}: lift returned None")
        xy_err = float(np.linalg.norm(obs.position_world[:2] - ball[:2]))
        z_err = float(abs(obs.position_world[2] - ball[2]))
        centre_err = float(np.hypot(u_det - u_gt, v_det - v_gt))
        return {
            "label": label,
            "centre_err_px": centre_err,
            "xy_err_mm": xy_err * 1000.0,
            "z_err_mm": z_err * 1000.0,
            "u_gt": u_gt,
            "v_gt": v_gt,
            "u_det": float(u_det),
            "v_det": float(v_det),
            "ball_world": ball.tolist(),
            "est_world": obs.position_world.tolist(),
            "image_path": "",
        }, capture.frame.image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("/tmp/fret_vision_bench"),
        help="Directory for JSON metrics, PNG frames, and bar chart",
    )
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

    from fret.mjcf.omx import ensure_omx_mjcf
    from fret.mjcf.omy import ensure_omy_mjcf

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)

    jobs = [
        (
            "omx_pick_place",
            ensure_omx_mjcf,
            "omx_pick_place",
            REPO / "src/fret/config/vision/omx_portal_overhead.yml",
        ),
        (
            "omy_pick_place",
            ensure_omy_mjcf,
            "omy_pick_place",
            REPO / "src/fret/config/vision/omy_portal_overhead.yml",
        ),
    ]

    rows: list[dict[str, float | str]] = []
    for label, ensure, scene, cfg in jobs:
        metrics, image = _run_one(
            label=label, ensure_fn=ensure, scene=scene, cfg_path=cfg
        )
        png = out / f"{label}_overhead.png"
        try:
            import imageio.v2 as imageio

            imageio.imwrite(png, image)
        except Exception:
            from PIL import Image

            Image.fromarray(image).save(png)
        metrics["image_path"] = str(png)
        rows.append(metrics)
        print(
            f"{label}: centre={metrics['centre_err_px']:.2f}px  "
            f"xy={metrics['xy_err_mm']:.2f}mm  z={metrics['z_err_mm']:.2f}mm"
        )

    (out / "metrics.json").write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf-8"
    )

    # Bar chart via OpenCV (avoids system matplotlib/NumPy skew).
    try:
        import cv2

        labels = [str(r["label"]).replace("_pick_place", "") for r in rows]
        centre = [float(r["centre_err_px"]) for r in rows]
        xy_mm = [float(r["xy_err_mm"]) for r in rows]
        width, height = 900, 360
        canvas = np.ones((height, width, 3), dtype=np.uint8) * 245

        def _panel(
            x0: int,
            title: str,
            values: list[float],
            gate_lines: list[tuple[float, tuple[int, int, int], str]],
            ymax: float,
            color: tuple[int, int, int],
        ) -> None:
            cv2.putText(
                canvas,
                title,
                (x0 + 20, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (30, 30, 30),
                1,
                cv2.LINE_AA,
            )
            px0, py0, pw, ph = x0 + 40, 50, 360, 250
            cv2.rectangle(
                canvas, (px0, py0), (px0 + pw, py0 + ph), (200, 200, 200), 1
            )
            bar_w = pw // (len(values) * 2)
            for i, (value, label) in enumerate(zip(values, labels)):
                bar_h = int(min(value / ymax, 1.0) * (ph - 10))
                x = px0 + (2 * i + 1) * bar_w
                y = py0 + ph - bar_h
                cv2.rectangle(canvas, (x, y), (x + bar_w, py0 + ph), color, -1)
                cv2.putText(
                    canvas,
                    label,
                    (x - 10, py0 + ph + 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (40, 40, 40),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    canvas,
                    f"{value:.2f}",
                    (x - 5, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (20, 20, 20),
                    1,
                    cv2.LINE_AA,
                )
            for gate, col, name in gate_lines:
                gy = py0 + ph - int(min(gate / ymax, 1.0) * (ph - 10))
                cv2.line(
                    canvas, (px0, gy), (px0 + pw, gy), col, 1, cv2.LINE_AA
                )
                cv2.putText(
                    canvas,
                    name,
                    (px0 + pw - 90, gy - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    col,
                    1,
                    cv2.LINE_AA,
                )

        _panel(
            0,
            "Image centre error (px)",
            centre,
            [(5.0, (40, 40, 200), "gate 5px")],
            6.0,
            (128, 90, 61),
        )
        _panel(
            450,
            "World XY error (mm)",
            xy_mm,
            [
                (15.0, (40, 40, 200), "OM-X 15"),
                (20.0, (80, 160, 240), "OMY 20"),
            ],
            25.0,
            (87, 129, 88),
        )
        cv2.putText(
            canvas,
            "MuJoCo portal HSV vision benchmark (v1.4 first PR)",
            (180, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
        chart = out / "portal_vision_errors.png"
        import imageio.v2 as imageio

        imageio.imwrite(chart, cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        print(f"wrote {chart}")
    except Exception as exc:  # pragma: no cover
        print(f"chart skipped: {exc}")

    print(f"wrote {out / 'metrics.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
