#!/usr/bin/env python3
"""Benchmark dual-gate HSV vision on MuJoCo OM-X / OMY pick-place cells.

Renders ``gate_cam_left`` / ``gate_cam_right``, runs ``fret.vision`` (detect +
table-plane fuse), and reports image-centre error vs geometric projection plus
world-frame pose error vs ``pick_box`` ground truth. Sweeps ball XY across the
cell (default pick, opposite side, front near cone, and a coarse grid).

Does **not** drive the pick-and-place FSM.

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
from typing import Sequence

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

_CAMS = ("gate_cam_left", "gate_cam_right")

# Nominal ball-centre Z from MJCF body pos (pedestal rest).
_BALL_Z = {
    "omx_pick_place": 0.1105,
    "omy_pick_place": 0.1330,
}

# Scenario anchors + coarse grid covering side pick (±Y) and front place (+X).
_SWEEPS: dict[str, list[tuple[str, float, float]]] = {
    "omx_pick_place": [
        ("pick_default", 0.22, -0.20),
        ("pick_opposite", 0.22, 0.20),
        ("front_cone", 0.26, 0.00),
        ("near_base_mY", 0.18, -0.18),
        ("near_base_pY", 0.18, 0.18),
        ("far_mY", 0.28, -0.22),
        ("far_pY", 0.28, 0.22),
        ("mid_front", 0.24, -0.10),
        ("mid_backish", 0.16, -0.12),
    ],
    "omy_pick_place": [
        ("pick_default", 0.35, -0.30),
        ("pick_opposite", 0.35, 0.30),
        ("front_cone", 0.48, 0.00),
        ("near_base_mY", 0.28, -0.25),
        ("near_base_pY", 0.28, 0.25),
        ("far_mY", 0.45, -0.32),
        ("far_pY", 0.45, 0.32),
        ("mid_front", 0.40, -0.15),
        ("mid_backish", 0.25, -0.18),
    ],
}


def _set_ball_xy(
    model: object,
    data: object,
    *,
    x: float,
    y: float,
    z: float,
) -> None:
    import mujoco

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    if jid < 0:
        raise RuntimeError("pick_box_joint missing")
    qadr = int(model.jnt_qposadr[jid])
    data.qpos[qadr : qadr + 3] = (x, y, z)
    data.qpos[qadr + 3 : qadr + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)


def _save_rgb(path: Path, rgb: np.ndarray) -> None:
    try:
        import imageio.v2 as imageio

        imageio.imwrite(path, rgb)
    except Exception:
        from PIL import Image

        Image.fromarray(rgb).save(path)


def _run_pose(
    *,
    label: str,
    pose_id: str,
    model: object,
    data: object,
    cfg_path: Path,
    ball_xy: tuple[float, float],
    ball_z: float,
    save_dir: Path | None,
) -> dict[str, float | str | int | list[float]]:
    import mujoco

    from fret.simulation.mujoco_camera import (
        MujocoCameraAdapter,
        project_world_point,
    )
    from fret.vision.config import load_hsv_plane_pipeline_config
    from fret.vision.detect.hsv_blob import HsvBlobBallDetector
    from fret.vision.factory import build_hsv_plane_pipeline

    _set_ball_xy(model, data, x=ball_xy[0], y=ball_xy[1], z=ball_z)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box")
    ball = np.asarray(data.xpos[body_id], dtype=np.float64).copy()

    cfg = load_hsv_plane_pipeline_config(cfg_path)
    pipeline = build_hsv_plane_pipeline(cfg_path)
    detector = HsvBlobBallDetector(cfg.detector)

    frames = []
    captures = []
    adapters = [
        MujocoCameraAdapter(model, data, camera_name=name) for name in _CAMS
    ]
    try:
        for adapter in adapters:
            cap = adapter.capture(timestamp=0.0)
            frames.append(cap.frame)
            captures.append(cap)
    finally:
        for adapter in adapters:
            adapter.close()

    detections = detector.detect(frames)
    obs = pipeline.process(frames)

    per_cam: dict[str, dict[str, float]] = {}
    centre_errors: list[float] = []
    for det in detections:
        cap = next(c for c in captures if c.frame.camera_id == det.camera_id)
        u_gt, v_gt = project_world_point(ball, cap.intrinsics, cap.extrinsics)
        err = float(np.hypot(det.centre_px[0] - u_gt, det.centre_px[1] - v_gt))
        centre_errors.append(err)
        per_cam[det.camera_id] = {
            "centre_err_px": err,
            "u_gt": float(u_gt),
            "v_gt": float(v_gt),
            "u_det": float(det.centre_px[0]),
            "v_det": float(det.centre_px[1]),
            "confidence": float(det.confidence),
        }

    if obs is None:
        xy_err_mm = float("nan")
        z_err_mm = float("nan")
        est_world: list[float] = []
        source = "none"
    else:
        xy_err_mm = float(
            np.linalg.norm(obs.position_world[:2] - ball[:2]) * 1000.0
        )
        z_err_mm = float(abs(obs.position_world[2] - ball[2]) * 1000.0)
        est_world = obs.position_world.tolist()
        source = str(obs.source)

    # Mono fallback: keep only the lower-confidence hit if both present.
    mono_xy_mm = float("nan")
    if len(detections) >= 2:
        kept = min(detections, key=lambda d: d.confidence)
        frames_one = [f for f in frames if f.camera_id == kept.camera_id]
        obs_one = pipeline.process(frames_one)
        if obs_one is not None:
            mono_xy_mm = float(
                np.linalg.norm(obs_one.position_world[:2] - ball[:2]) * 1000.0
            )

    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        for cap in captures:
            _save_rgb(
                save_dir / f"{label}_{pose_id}_{cap.frame.camera_id}.png",
                cap.frame.image,
            )

    return {
        "label": label,
        "pose_id": pose_id,
        "ball_xy": [float(ball_xy[0]), float(ball_xy[1])],
        "ball_world": ball.tolist(),
        "n_detections": int(len(detections)),
        "cameras_hit": sorted(per_cam.keys()),
        "centre_err_px_max": (
            float(max(centre_errors)) if centre_errors else float("nan")
        ),
        "centre_err_px_mean": (
            float(np.mean(centre_errors)) if centre_errors else float("nan")
        ),
        "xy_err_mm": xy_err_mm,
        "z_err_mm": z_err_mm,
        "mono_xy_err_mm": mono_xy_mm,
        "source": source,
        "est_world": est_world,
        "per_cam": per_cam,
    }


def _summary(
    rows: Sequence[dict[str, float | str | int | list[float]]],
) -> dict:
    def _finite(key: str) -> list[float]:
        vals = []
        for r in rows:
            v = float(r[key])  # type: ignore[arg-type]
            if np.isfinite(v):
                vals.append(v)
        return vals

    xy = _finite("xy_err_mm")
    z = _finite("z_err_mm")
    px = _finite("centre_err_px_max")
    mono = _finite("mono_xy_err_mm")
    n_hit = [int(r["n_detections"]) for r in rows]  # type: ignore[arg-type]
    return {
        "n_poses": len(rows),
        "n_detected": sum(1 for r in rows if int(r["n_detections"]) >= 1),  # type: ignore[arg-type]
        "n_dual_view": sum(1 for n in n_hit if n >= 2),
        "n_mono_view": sum(1 for n in n_hit if n == 1),
        "n_miss": sum(1 for n in n_hit if n == 0),
        "centre_err_px_max": float(max(px)) if px else None,
        "centre_err_px_mean": float(np.mean(px)) if px else None,
        "xy_err_mm_max": float(max(xy)) if xy else None,
        "xy_err_mm_mean": float(np.mean(xy)) if xy else None,
        "xy_err_mm_p95": float(np.percentile(xy, 95)) if xy else None,
        "z_err_mm_max": float(max(z)) if z else None,
        "z_err_mm_mean": float(np.mean(z)) if z else None,
        "mono_xy_err_mm_max": float(max(mono)) if mono else None,
        "mono_xy_err_mm_mean": float(np.mean(mono)) if mono else None,
    }


def _write_charts(
    out: Path,
    by_label: dict[str, list[dict]],
    summaries: dict[str, dict],
) -> None:
    import cv2

    # --- Summary bars (default pick + sweep max/mean) ---
    labels = list(by_label.keys())
    short = [lab.replace("_pick_place", "") for lab in labels]
    mean_xy = [
        float(summaries[lab]["xy_err_mm_mean"] or 0.0) for lab in labels
    ]
    max_xy = [float(summaries[lab]["xy_err_mm_max"] or 0.0) for lab in labels]
    mean_px = [
        float(summaries[lab]["centre_err_px_mean"] or 0.0) for lab in labels
    ]
    max_px = [
        float(summaries[lab]["centre_err_px_max"] or 0.0) for lab in labels
    ]

    width, height = 1000, 420
    canvas = np.ones((height, width, 3), dtype=np.uint8) * 245

    def _panel(
        x0: int,
        title: str,
        means: list[float],
        maxes: list[float],
        gate_lines: list[tuple[float, tuple[int, int, int], str]],
        ymax: float,
    ) -> None:
        cv2.putText(
            canvas,
            title,
            (x0 + 16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )
        px0, py0, pw, ph = x0 + 40, 50, 420, 300
        cv2.rectangle(
            canvas, (px0, py0), (px0 + pw, py0 + ph), (200, 200, 200), 1
        )
        group = pw // max(len(means), 1)
        bar_w = max(group // 5, 18)
        for i, (mn, mx, lab) in enumerate(zip(means, maxes, short)):
            x = px0 + i * group + group // 4
            for value, color, dx in (
                (mn, (87, 129, 88), 0),
                (mx, (61, 90, 128), bar_w + 4),
            ):
                bar_h = int(min(value / ymax, 1.0) * (ph - 10))
                y = py0 + ph - bar_h
                cv2.rectangle(
                    canvas,
                    (x + dx, y),
                    (x + dx + bar_w, py0 + ph),
                    color,
                    -1,
                )
                cv2.putText(
                    canvas,
                    f"{value:.2f}",
                    (x + dx - 4, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (20, 20, 20),
                    1,
                    cv2.LINE_AA,
                )
            cv2.putText(
                canvas,
                lab,
                (x, py0 + ph + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (40, 40, 40),
                1,
                cv2.LINE_AA,
            )
        for gate, col, name in gate_lines:
            gy = py0 + ph - int(min(gate / ymax, 1.0) * (ph - 10))
            cv2.line(canvas, (px0, gy), (px0 + pw, gy), col, 1, cv2.LINE_AA)
            cv2.putText(
                canvas,
                name,
                (px0 + pw - 100, gy - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                col,
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            canvas,
            "mean",
            (px0, py0 + ph + 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (87, 129, 88),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "max",
            (px0 + 55, py0 + ph + 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (61, 90, 128),
            1,
            cv2.LINE_AA,
        )

    _panel(
        0,
        "Image centre error (px, max over hitting cams)",
        mean_px,
        max_px,
        [(5.0, (40, 40, 200), "gate 5px")],
        6.0,
    )
    _panel(
        500,
        "World XY error (mm, fused)",
        mean_xy,
        max_xy,
        [
            (15.0, (40, 40, 200), "OM-X 15"),
            (20.0, (80, 160, 240), "OMY 20"),
        ],
        25.0,
    )
    cv2.putText(
        canvas,
        "MuJoCo gate dual-cam HSV exteroception sweep (v1.4)",
        (200, height - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (20, 20, 20),
        1,
        cv2.LINE_AA,
    )
    chart = out / "gate_vision_errors.png"
    _save_rgb(chart, cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    print(f"wrote {chart}")

    # --- Per-pose XY scatter for each robot ---
    for lab, rows in by_label.items():
        w, h = 720, 560
        img = np.ones((h, w, 3), dtype=np.uint8) * 250
        xs = [float(r["ball_xy"][0]) for r in rows]  # type: ignore[index]
        ys = [float(r["ball_xy"][1]) for r in rows]  # type: ignore[index]
        errs = [float(r["xy_err_mm"]) for r in rows]
        xmin, xmax = min(xs) - 0.05, max(xs) + 0.05
        ymin, ymax = min(ys) - 0.05, max(ys) + 0.05
        err_max = max(max(errs), 1e-3)

        def to_px(x: float, y: float) -> tuple[int, int]:
            u = int(60 + (x - xmin) / (xmax - xmin) * (w - 120))
            v = int(60 + (1.0 - (y - ymin) / (ymax - ymin)) * (h - 140))
            return u, v

        cv2.putText(
            img,
            f"{lab}: XY error (mm) vs ball pose",
            (40, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
        # Axes cross at origin-ish of plot region.
        for r, e, x, y in zip(rows, errs, xs, ys):
            u, v = to_px(x, y)
            radius = 8 + int(14 * min(e / err_max, 1.0))
            # Green→amber by error magnitude (still ≪ gate).
            t = min(e / 5.0, 1.0)
            color = (
                int(80 + 100 * t),
                int(180 - 80 * t),
                int(80),
            )
            cv2.circle(img, (u, v), radius, color, -1, cv2.LINE_AA)
            cv2.putText(
                img,
                f"{e:.2f}",
                (u + 10, v - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (30, 30, 30),
                1,
                cv2.LINE_AA,
            )
            n = int(r["n_detections"])
            cv2.putText(
                img,
                f"{r['pose_id']} ({n}v)",
                (u + 10, v + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (90, 90, 90),
                1,
                cv2.LINE_AA,
            )
        path = out / f"{lab}_xy_error_map.png"
        _save_rgb(path, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        print(f"wrote {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("/tmp/fret_vision_bench"),
        help="Directory for JSON metrics, PNG frames, and charts",
    )
    parser.add_argument(
        "--save-frames",
        action="store_true",
        help="Write per-pose gate camera RGB frames",
    )
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

    import mujoco

    from fret.mjcf.omx import ensure_omx_mjcf
    from fret.mjcf.omy import ensure_omy_mjcf

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    frames_dir = out / "frames" if args.save_frames else None

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

    by_label: dict[str, list[dict]] = {}
    all_rows: list[dict] = []
    for label, ensure, scene, cfg in jobs:
        xml = ensure(scene)
        model = mujoco.MjModel.from_xml_path(str(xml))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ball_z = _BALL_Z[scene]
        rows: list[dict] = []
        for pose_id, x, y in _SWEEPS[scene]:
            metrics = _run_pose(
                label=label,
                pose_id=pose_id,
                model=model,
                data=data,
                cfg_path=cfg,
                ball_xy=(x, y),
                ball_z=ball_z,
                save_dir=frames_dir,
            )
            rows.append(metrics)
            all_rows.append(metrics)
            print(
                f"{label}/{pose_id}: views={metrics['n_detections']}  "
                f"centre_max={metrics['centre_err_px_max']:.2f}px  "
                f"xy={metrics['xy_err_mm']:.3f}mm  "
                f"z={metrics['z_err_mm']:.3f}mm  "
                f"mono={metrics['mono_xy_err_mm']}"
            )
        by_label[label] = rows

    summaries = {lab: _summary(rows) for lab, rows in by_label.items()}
    payload = {
        "cameras": list(_CAMS),
        "summaries": summaries,
        "rows": all_rows,
    }
    (out / "metrics.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summaries, indent=2))

    try:
        _write_charts(out, by_label, summaries)
    except Exception as exc:  # pragma: no cover
        print(f"chart skipped: {exc}")

    # Markdown report snippet.
    lines = [
        "# Gate dual-cam exteroception re-benchmark",
        "",
        "Sweep of ball XY on OM-X / OMY pick-place after rear-gate redesign.",
        "Pipeline: HSV blob → table-plane lift → confidence-weighted fuse.",
        "",
        "| Robot | Poses | Dual | Mono | Miss | XY mean / max (mm) | XY p95 | Centre mean / max (px) | Mono XY max (mm) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for lab, s in summaries.items():
        lines.append(
            f"| {lab} | {s['n_poses']} | {s['n_dual_view']} | {s['n_mono_view']} | "
            f"{s['n_miss']} | "
            f"{s['xy_err_mm_mean']:.3f} / {s['xy_err_mm_max']:.3f} | "
            f"{s['xy_err_mm_p95']:.3f} | "
            f"{s['centre_err_px_mean']:.3f} / {s['centre_err_px_max']:.3f} | "
            f"{(s['mono_xy_err_mm_max'] or float('nan')):.3f} |"
        )
    lines.extend(
        [
            "",
            "Gates: centre ≤ 5 px; XY ≤ 15 mm (OM-X) / 20 mm (OMY); Z ≤ 5 mm.",
            "",
            "Artifacts: `gate_vision_errors.png`, `*_xy_error_map.png`, `metrics.json`.",
            "",
        ]
    )
    report = out / "REPORT.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {report}")
    print(f"wrote {out / 'metrics.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
