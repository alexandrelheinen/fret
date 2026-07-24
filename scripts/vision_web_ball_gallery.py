#!/usr/bin/env python3
"""Download web ball photos, run HSV vision MVP, annotate 16:9 panels, optional R2.

Reads a YAML image list (see ``src/fret/config/vision/demo_web_balls.yml``),
runs :func:`fret.vision.build_hsv_plane_pipeline` with an arbitrary / demo
calibration, draws OpenCV overlays (mask tint + detection circle), lays out
**2/3 image + 1/3 text** on a 16:9 canvas, and optionally uploads PNGs to
Cloudflare R2 (same credentials as showcase scripts).

Examples::

    python3 scripts/vision_web_ball_gallery.py
    python3 scripts/vision_web_ball_gallery.py --upload-r2
    python3 scripts/vision_web_ball_gallery.py \\
        --images src/fret/config/vision/demo_web_balls.yml \\
        --out /tmp/fret_vision_gallery
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "opencv-python-headless is required "
        "(pip install '.[vision]' or '.[sim]')"
    ) from exc

from fret.sitl_config import resolve_package_file
from fret.vision.config import load_hsv_plane_pipeline_config
from fret.vision.detect.hsv_blob import HsvBlobBallDetector
from fret.vision.factory import pipeline_from_hsv_plane_config
from fret.vision.geometry.plane_lifter import look_at_extrinsics
from fret.vision.types import (
    BallDetection,
    BallObservation,
    CameraFrame,
    CameraIntrinsics,
    VisionConfig,
)

_USER_AGENT = (
    "FRET-vision-web-gallery/1.0 "
    "(+https://github.com/alexandrelheinen/fret; research demo)"
)


def _load_dotenv(path: Path) -> None:
    if not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, _, value = stripped.partition("=")
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        os.environ.setdefault(key, value)


def _load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def _resolve_vision_config_path(
    images_yaml: dict[str, Any], override: Path | None
) -> Path:
    if override is not None:
        return override
    rel = images_yaml.get("vision_config", "vision/hsv_blob_tennis_yellow.yml")
    if isinstance(rel, str) and not Path(rel).is_file():
        return resolve_package_file("config", *rel.split("/"))
    return Path(str(rel))


def _download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request(url, headers={"User-Agent": _USER_AGENT})
    with urllib.request.urlopen(req, timeout=60) as resp:
        dest.write_bytes(resp.read())


def _load_image_bgr(path: Path) -> npt.NDArray[np.uint8]:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"failed to decode image: {path}")
    return np.asarray(image, dtype=np.uint8)


def _bgr_to_rgb(image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
    return np.asarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), dtype=np.uint8)


def _letterbox(
    image_bgr: npt.NDArray[np.uint8],
    out_w: int,
    out_h: int,
    *,
    pad_color: tuple[int, int, int] = (24, 24, 24),
) -> tuple[npt.NDArray[np.uint8], float, int, int]:
    """Fit ``image_bgr`` into ``out_w×out_h``; return canvas, scale, ox, oy."""
    h, w = image_bgr.shape[:2]
    scale = min(out_w / float(w), out_h / float(h))
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    resized = cv2.resize(image_bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.full((out_h, out_w, 3), pad_color, dtype=np.uint8)
    ox = (out_w - nw) // 2
    oy = (out_h - nh) // 2
    canvas[oy : oy + nh, ox : ox + nw] = resized
    return canvas, scale, ox, oy


def _annotate_detection(
    image_bgr: npt.NDArray[np.uint8],
    mask: npt.NDArray[np.uint8],
    detection: BallDetection | None,
) -> npt.NDArray[np.uint8]:
    """Tint mask green and draw detection circle / crosshair."""
    out = image_bgr.copy()
    tint = out.copy()
    tint[mask > 0] = (40, 200, 40)
    cv2.addWeighted(tint, 0.35, out, 0.65, 0.0, dst=out)
    if detection is not None:
        cx, cy = int(round(detection.centre_px[0])), int(
            round(detection.centre_px[1])
        )
        radius = max(1, int(round(detection.radius_px)))
        cv2.circle(out, (cx, cy), radius, (0, 255, 255), 2)
        cv2.drawMarker(
            out,
            (cx, cy),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
    return out


def _wrap_text(text: str, max_chars: int) -> list[str]:
    words = text.split()
    if not words:
        return [""]
    lines: list[str] = []
    current = words[0]
    for word in words[1:]:
        candidate = f"{current} {word}"
        if len(candidate) <= max_chars:
            current = candidate
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines


def _draw_text_panel(
    width: int,
    height: int,
    lines: list[str],
    *,
    title: str,
) -> npt.NDArray[np.uint8]:
    panel = np.full((height, width, 3), (18, 18, 22), dtype=np.uint8)
    y = 48
    cv2.putText(
        panel,
        title[:60],
        (24, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (230, 230, 230),
        2,
        cv2.LINE_AA,
    )
    y += 36
    cv2.line(panel, (24, y), (width - 24, y), (60, 60, 70), 1)
    y += 40
    max_chars = max(24, width // 11)
    for raw in lines:
        for part in _wrap_text(raw, max_chars):
            cv2.putText(
                panel,
                part,
                (24, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )
            y += 28
            if y > height - 24:
                return panel
        y += 8
    return panel


def _compose_16x9(
    annotated_bgr: npt.NDArray[np.uint8],
    text_lines: list[str],
    *,
    canvas_w: int,
    canvas_h: int,
    title: str,
) -> npt.NDArray[np.uint8]:
    img_w = (canvas_w * 2) // 3
    text_w = canvas_w - img_w
    left, _scale, _ox, _oy = _letterbox(annotated_bgr, img_w, canvas_h)
    right = _draw_text_panel(text_w, canvas_h, text_lines, title=title)
    return np.concatenate([left, right], axis=1)


def _intrinsics_for_image(
    camera_id: str, width: int, height: int
) -> CameraIntrinsics:
    """Arbitrary pinhole: fx≈width, principal point at centre."""
    fx = float(width)
    fy = float(width)
    return CameraIntrinsics(
        camera_id=camera_id,
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=width * 0.5,
        cy=height * 0.5,
    )


def _build_pipeline_for_image(
    vision_cfg_path: Path, width: int, height: int
) -> tuple[Any, HsvBlobBallDetector]:
    base = load_hsv_plane_pipeline_config(vision_cfg_path)
    camera_id = base.detector.camera_id
    intr = _intrinsics_for_image(camera_id, width, height)
    # Nadir-ish look-at 1.2 m above origin (demo only — not metric GT).
    ext = look_at_extrinsics(
        camera_id=camera_id,
        eye_world=(0.0, 0.0, 1.2),
        target_world=(0.0, 0.0, 0.0),
    )
    from fret.vision.config import HsvPlanePipelineConfig

    wired = HsvPlanePipelineConfig(
        vision=VisionConfig(intrinsics=(intr,), extrinsics=(ext,)),
        detector=base.detector,
        lifter=base.lifter,
        source=base.source,
    )
    pipe = pipeline_from_hsv_plane_config(wired)
    detector = HsvBlobBallDetector(base.detector)
    return pipe, detector


def _format_result_lines(
    *,
    entry: dict[str, Any],
    detection: BallDetection | None,
    observation: BallObservation | None,
    image_shape: tuple[int, ...],
    vision_cfg: Path,
) -> list[str]:
    h, w = int(image_shape[0]), int(image_shape[1])
    lines = [
        f"id: {entry.get('id', '?')}",
        f"input: {w}×{h} px",
        f"vision YAML: {vision_cfg.name}",
        f"calib: arbitrary look-at @ z=1.2 m (demo)",
        "",
    ]
    if detection is None:
        lines.append("detection: NONE")
        lines.append("observation: n/a")
    else:
        lines.extend(
            [
                "detection: OK",
                f"  centre_px: ({detection.centre_px[0]:.1f}, "
                f"{detection.centre_px[1]:.1f})",
                f"  radius_px: {detection.radius_px:.1f}",
                f"  confidence: {detection.confidence:.3f}",
            ]
        )
        if observation is None:
            lines.append("observation: lift failed")
        else:
            p = observation.position_world
            lines.extend(
                [
                    "observation: OK",
                    f"  xyz_m: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})",
                    f"  radius_m: {observation.radius_m:.4f}",
                    f"  source: {observation.source}",
                ]
            )
    notes = str(entry.get("notes") or "").strip()
    if notes:
        lines.extend(["", f"notes: {notes}"])
    src = str(entry.get("source_page") or entry.get("url") or "").strip()
    if src:
        lines.extend(["", f"credit: {src}"])
    return lines


def _upload_r2(
    local_path: Path,
    *,
    key: str,
    endpoint: str,
    bucket: str,
    content_type: str = "image/png",
) -> None:
    cmd = [
        "aws",
        "s3",
        "cp",
        str(local_path),
        f"s3://{bucket}/{key}",
        "--endpoint-url",
        endpoint,
        "--content-type",
        content_type,
    ]
    subprocess.run(cmd, check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--images",
        type=Path,
        default=REPO_ROOT / "src/fret/config/vision/demo_web_balls.yml",
        help="YAML list of image URLs / paths",
    )
    parser.add_argument(
        "--vision-config",
        type=Path,
        default=None,
        help="Override HSV/plane YAML (default from images YAML)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("/tmp/fret_vision_web_gallery"),
        help="Output directory for annotated PNGs",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=Path("/tmp/fret_vision_web_cache"),
        help="Download cache for remote URLs",
    )
    parser.add_argument(
        "--upload-r2",
        action="store_true",
        help="Upload PNGs to Cloudflare R2 (needs .env / R2_* env vars)",
    )
    parser.add_argument(
        "--r2-prefix",
        type=str,
        default=None,
        help="Override R2 key prefix (default from images YAML)",
    )
    args = parser.parse_args(argv)

    _load_dotenv(REPO_ROOT / ".env")
    images_yaml = _load_yaml(args.images)
    vision_cfg_path = _resolve_vision_config_path(
        images_yaml, args.vision_config
    )
    canvas = images_yaml.get("canvas") or {}
    canvas_w = int(canvas.get("width", 1920))
    canvas_h = int(canvas.get("height", 1080))
    if abs(canvas_w / canvas_h - 16 / 9) > 0.05:
        print(
            f"[WARN] canvas {canvas_w}x{canvas_h} is not ~16:9",
            file=sys.stderr,
        )

    entries = images_yaml.get("images")
    if not isinstance(entries, list) or not entries:
        raise SystemExit("images YAML has empty 'images' list")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    r2_cfg = images_yaml.get("r2") or {}
    r2_prefix = args.r2_prefix or str(
        r2_cfg.get("prefix", "evidence/vision_web_balls")
    )
    r2_prefix = r2_prefix.rstrip("/") + f"/{stamp}"

    summary: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        image_id = str(entry.get("id") or "unnamed")
        title = str(entry.get("title") or image_id)
        local_path: Path | None = None
        if entry.get("path"):
            local_path = Path(str(entry["path"]))
            if not local_path.is_file():
                local_path = REPO_ROOT / local_path
        elif entry.get("url"):
            url = str(entry["url"])
            suffix = Path(url.split("?", 1)[0]).suffix or ".jpg"
            local_path = args.cache_dir / f"{image_id}{suffix}"
            if not local_path.is_file():
                print(f"[INFO] downloading {image_id} …")
                try:
                    _download(url, local_path)
                except (urllib.error.URLError, TimeoutError) as err:
                    print(
                        f"[FAIL] download {image_id}: {err}", file=sys.stderr
                    )
                    summary.append(f"{image_id}: DOWNLOAD_FAIL")
                    continue
        else:
            print(f"[FAIL] {image_id}: need url or path", file=sys.stderr)
            summary.append(f"{image_id}: NO_SOURCE")
            continue

        assert local_path is not None
        try:
            bgr = _load_image_bgr(local_path)
        except ValueError as err:
            print(f"[FAIL] {image_id}: {err}", file=sys.stderr)
            summary.append(f"{image_id}: DECODE_FAIL")
            continue

        h, w = bgr.shape[:2]
        pipe, detector = _build_pipeline_for_image(vision_cfg_path, w, h)
        rgb = _bgr_to_rgb(bgr)
        frame = CameraFrame(
            camera_id=detector.config.camera_id,
            image=rgb,
            timestamp=0.0,
            intrinsics_id=detector.config.camera_id,
        )
        mask = detector.mask_for_frame(frame)
        detections = detector.detect([frame])
        detection = detections[0] if detections else None
        observation = pipe.process([frame])
        annotated = _annotate_detection(bgr, mask, detection)
        text_lines = _format_result_lines(
            entry=entry,
            detection=detection,
            observation=observation,
            image_shape=bgr.shape,
            vision_cfg=vision_cfg_path,
        )
        panel = _compose_16x9(
            annotated,
            text_lines,
            canvas_w=canvas_w,
            canvas_h=canvas_h,
            title=title,
        )
        out_path = out_dir / f"{image_id}_panel.png"
        ok = cv2.imwrite(str(out_path), panel)
        if not ok:
            print(f"[FAIL] write {out_path}", file=sys.stderr)
            summary.append(f"{image_id}: WRITE_FAIL")
            continue
        status = "HIT" if detection is not None else "MISS"
        print(f"[ OK ] {image_id}: {status} → {out_path}")
        summary.append(f"{image_id}: {status}")

        if args.upload_r2:
            account = os.environ.get("R2_ACCOUNT_ID", "")
            bucket = os.environ.get("R2_BUCKET", "")
            key_id = os.environ.get("R2_ACCESS_KEY_ID") or os.environ.get(
                "AWS_ACCESS_KEY_ID", ""
            )
            secret = os.environ.get("R2_SECRET_ACCESS_KEY") or os.environ.get(
                "AWS_SECRET_ACCESS_KEY", ""
            )
            if not (account and bucket and key_id and secret):
                raise SystemExit(
                    "--upload-r2 needs R2_ACCOUNT_ID, R2_BUCKET, "
                    "R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY "
                    "(e.g. copy .env.example → .env)"
                )
            os.environ["AWS_ACCESS_KEY_ID"] = key_id
            os.environ["AWS_SECRET_ACCESS_KEY"] = secret
            endpoint = f"https://{account}.r2.cloudflarestorage.com"
            key = f"{r2_prefix}/{out_path.name}"
            print(f"[INFO] uploading → s3://{bucket}/{key}")
            _upload_r2(out_path, key=key, endpoint=endpoint, bucket=bucket)

    manifest = out_dir / "summary.txt"
    manifest.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(f"[INFO] wrote {manifest}")
    if args.upload_r2:
        account = os.environ["R2_ACCOUNT_ID"]
        bucket = os.environ["R2_BUCKET"]
        endpoint = f"https://{account}.r2.cloudflarestorage.com"
        _upload_r2(
            manifest,
            key=f"{r2_prefix}/summary.txt",
            endpoint=endpoint,
            bucket=bucket,
            content_type="text/plain",
        )
        print(f"[INFO] R2 prefix: {r2_prefix}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
