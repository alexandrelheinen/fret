#!/usr/bin/env python3
"""Generate figures and numeric tables for the vision-algorithm PDF.

Renders the OM-X / OMY gate cameras in MuJoCo, runs the shipped
``fret.vision`` HSV + table-plane pipeline, and checks every closed-form
result used in ``docs/vision/algorithm/vision_algorithm.tex``
(perspective bias of the sphere silhouette, pixel-to-world Jacobian,
minimum-enclosing-circle behaviour under occlusion, pad-mid IK residuals).

Outputs (overwritten):

* ``docs/vision/algorithm/figures/*.png``
* ``docs/vision/algorithm/data.tex`` (LaTeX macros) and ``sweep_*.tex``
* ``docs/vision/algorithm/vision_algorithm.md`` rendered from
  ``vision_algorithm.template.md`` with the same numbers
* ``docs/vision/algorithm/metrics.json``

Example::

    MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \\
      python3 scripts/vision_algorithm_figures.py
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

DOC_DIR = REPO / "docs" / "vision" / "algorithm"
FIG_DIR = DOC_DIR / "figures"

CAMS = ("gate_cam_left", "gate_cam_right")
CAM_LABEL = {"gate_cam_left": "left", "gate_cam_right": "right"}

Array = npt.NDArray[np.float64]

ROBOTS: dict[str, dict[str, Any]] = {
    "omx": {
        "model": "open_manipulator_x",
        "scenario": "omx_pick_place",
        "scenario_yml": "src/fret/config/scenarios/omx_pick_place.yml",
        "vision_yml": "src/fret/config/vision/omx_portal_overhead.yml",
        "ball_z": 0.020,
        "title": "OpenMANIPULATOR-X",
        "sweep": [
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
        "grid_x": (0.08, 0.34, 0.02),
        "grid_y": (-0.30, 0.30, 0.04),
    },
    "omy": {
        "model": "omy",
        "scenario": "omy_pick_place",
        "scenario_yml": "src/fret/config/scenarios/omy_pick_place.yml",
        "vision_yml": "src/fret/config/vision/omy_portal_overhead.yml",
        "ball_z": 0.043,
        "title": "OpenMANIPULATOR-Y",
        "sweep": [
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
        "grid_x": (0.14, 0.60, 0.035),
        "grid_y": (-0.42, 0.42, 0.06),
    },
}


# --------------------------------------------------------------------------
# Scene / rendering helpers
# --------------------------------------------------------------------------


@dataclass
class Scene:
    """Live MuJoCo model plus the vision config for one robot cell."""

    robot: str
    model: Any
    data: Any
    cfg: Any
    pipeline: Any
    detector: Any
    adapters: dict[str, Any]
    seg_renderer: Any
    ball_geom: int
    ball_body: int
    ball_qadr: int
    spec: dict[str, Any] = field(default_factory=dict)

    def close(self) -> None:
        for adapter in self.adapters.values():
            adapter.close()
        self.seg_renderer.close()


def load_scene(robot: str) -> Scene:
    import mujoco

    from fret.simulation.mujoco_camera import MujocoCameraAdapter
    from fret.sitl_config import mjcf_path
    from fret.vision.config import load_hsv_plane_pipeline_config
    from fret.vision.detect.hsv_blob import HsvBlobBallDetector
    from fret.vision.factory import pipeline_from_hsv_plane_config

    spec = ROBOTS[robot]
    xml = mjcf_path(spec["model"], spec["scenario"])
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cfg = load_hsv_plane_pipeline_config(REPO / spec["vision_yml"])
    adapters = {
        c: MujocoCameraAdapter(model, data, camera_name=c) for c in CAMS
    }
    seg = mujoco.Renderer(model, height=720, width=1280)
    seg.enable_segmentation_rendering()
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    return Scene(
        robot=robot,
        model=model,
        data=data,
        cfg=cfg,
        pipeline=pipeline_from_hsv_plane_config(cfg),
        detector=HsvBlobBallDetector(cfg.detector),
        adapters=adapters,
        seg_renderer=seg,
        ball_geom=mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "pick_box_geom"
        ),
        ball_body=mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "pick_box"
        ),
        ball_qadr=int(model.jnt_qposadr[jid]),
        spec=spec,
    )


def set_ball(scene: Scene, x: float, y: float) -> Array:
    import mujoco

    q = scene.ball_qadr
    scene.data.qpos[q : q + 3] = (x, y, scene.spec["ball_z"])
    scene.data.qpos[q + 3 : q + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(scene.model, scene.data)
    return np.asarray(
        scene.data.xpos[scene.ball_body], dtype=np.float64
    ).copy()


@dataclass
class View:
    camera_id: str
    capture: Any
    ball_seg: npt.NDArray[np.bool_]


def render_views(scene: Scene, *, with_seg: bool = True) -> list[View]:
    import mujoco

    views = []
    for cam in CAMS:
        cap = scene.adapters[cam].capture(timestamp=0.0)
        if with_seg:
            scene.seg_renderer.update_scene(scene.data, camera=cam)
            seg = scene.seg_renderer.render()
            ball = (seg[..., 0] == scene.ball_geom) & (
                seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
            )
        else:
            ball = np.zeros(cap.frame.image.shape[:2], dtype=bool)
        views.append(View(camera_id=cam, capture=cap, ball_seg=ball))
    return views


# --------------------------------------------------------------------------
# Math used in the document (mirrors fret.vision, checked against it)
# --------------------------------------------------------------------------


def mask_stages(
    rgb: npt.NDArray[np.uint8], det_cfg: Any
) -> dict[str, npt.NDArray[np.uint8]]:
    import cv2

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(
        hsv,
        np.asarray(det_cfg.hsv_lower, dtype=np.uint8),
        np.asarray(det_cfg.hsv_upper, dtype=np.uint8),
    )
    k = int(det_cfg.morph_kernel_px)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    opened = cv2.morphologyEx(raw, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    return {
        "hsv": hsv,
        "raw": raw,
        "opened": opened,
        "closed": closed,
        "kernel": kernel,
    }


def contour_stats(mask: npt.NDArray[np.uint8], det_cfg: Any) -> dict[str, Any]:
    """Re-run the detector's contour scoring and expose every quantity."""
    import cv2

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    rows = []
    for c in contours:
        area = float(cv2.contourArea(c))
        perim = float(cv2.arcLength(c, True))
        circ = 4.0 * np.pi * area / (perim * perim) if perim > 1e-9 else 0.0
        (cx, cy), rad = cv2.minEnclosingCircle(c)
        ok = (
            det_cfg.min_area_px <= area <= det_cfg.max_area_px
            and circ >= det_cfg.min_circularity
        )
        score = circ + 0.05 * min(1.0, area / max(det_cfg.max_area_px, 1.0))
        m = cv2.moments(c)
        centroid = (
            (m["m10"] / m["m00"], m["m01"] / m["m00"])
            if m["m00"] > 0
            else (float("nan"), float("nan"))
        )
        rows.append(
            {
                "contour": c,
                "area": area,
                "perimeter": perim,
                "circularity": circ,
                "centre": (float(cx), float(cy)),
                "radius": float(rad),
                "accepted": bool(ok),
                "score": float(score),
                "centroid": centroid,
                "n_vertices": int(len(c)),
            }
        )
    accepted = [r for r in rows if r["accepted"]]
    best = max(accepted, key=lambda r: r["score"]) if accepted else None
    return {"all": rows, "best": best}


def ray_world(u: float, v: float, intr: Any, extr: Any) -> tuple[Array, Array]:
    """Return (origin, unnormalised direction) of the pixel ray in world."""
    d_cam = np.array(
        [(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0],
        dtype=np.float64,
    )
    rot = extr.t_world_cam[:3, :3]
    return extr.t_world_cam[:3, 3].copy(), rot @ d_cam


def lift_pixel(
    u: float, v: float, intr: Any, extr: Any, plane_z: float
) -> Array | None:
    origin, d = ray_world(u, v, intr, extr)
    if abs(d[2]) < 1e-12:
        return None
    t = (plane_z - origin[2]) / d[2]
    if t <= 0.0:
        return None
    return origin + t * d


def lift_jacobian(
    u: float, v: float, intr: Any, extr: Any, plane_z: float
) -> Array:
    """Analytic d(x, y)/d(u, v) of the ray-plane intersection (2x2)."""
    origin, d = ray_world(u, v, intr, extr)
    rot = extr.t_world_cam[:3, :3]
    t = (plane_z - origin[2]) / d[2]
    cols = []
    for k, f in ((0, intr.fx), (1, intr.fy)):
        r_k = rot[:, k]
        dp = (t / f) * (r_k - (r_k[2] / d[2]) * d)
        cols.append(dp[:2])
    return np.column_stack(cols)


def project(point: Array, intr: Any, extr: Any) -> tuple[float, float, float]:
    rot = extr.t_world_cam[:3, :3]
    pc = rot.T @ (
        np.asarray(point, dtype=np.float64) - extr.t_world_cam[:3, 3]
    )
    return (
        intr.fx * pc[0] / pc[2] + intr.cx,
        intr.fy * pc[1] / pc[2] + intr.cy,
        float(pc[2]),
    )


def sphere_ellipse(
    centre_world: Array, radius: float, intr: Any, extr: Any
) -> dict[str, float]:
    """Closed-form silhouette ellipse of a sphere (normalised -> pixels)."""
    rot = extr.t_world_cam[:3, :3]
    pc = rot.T @ (centre_world - extr.t_world_cam[:3, 3])
    dist = float(np.linalg.norm(pc))
    alpha = float(np.arccos(pc[2] / dist))
    beta = float(np.arcsin(radius / dist))
    a_cap = np.cos(beta) ** 2 - np.sin(alpha) ** 2
    semi_major = np.sin(beta) * np.cos(beta) / a_cap
    semi_minor = np.sin(beta) / np.sqrt(a_cap)
    delta = np.tan(alpha) * np.sin(beta) ** 2 / a_cap
    radial = pc[:2] / max(float(np.linalg.norm(pc[:2])), 1e-15)
    u0, v0, _ = project(centre_world, intr, extr)
    return {
        "dist": dist,
        "alpha": alpha,
        "beta": beta,
        "a_px": float(intr.fx * semi_major),
        "b_px": float(intr.fx * semi_minor),
        "bias_px": float(intr.fx * delta),
        "u_centre": float(u0 + intr.fx * delta * radial[0]),
        "v_centre": float(v0 + intr.fy * delta * radial[1]),
        "area_px": float(np.pi * intr.fx * intr.fy * semi_major * semi_minor),
    }


# --------------------------------------------------------------------------
# LaTeX output helpers
# --------------------------------------------------------------------------


class Macros:
    """Collect ``\\newcommand`` lines for ``data.tex``."""

    def __init__(self) -> None:
        self.lines: list[str] = [
            "% Generated by scripts/vision_algorithm_figures.py -- do not edit.",
        ]
        self.data: dict[str, Any] = {}
        self.text: dict[str, str] = {}

    def add(self, name: str, value: float | str, fmt: str = "{:.3f}") -> None:
        text = value if isinstance(value, str) else fmt.format(value)
        text = text.replace("nan", "--")
        self.lines.append(f"\\newcommand{{\\{name}}}{{{text}}}")
        self.data[name] = value if isinstance(value, str) else float(value)
        self.text[name] = text

    def raw(self, text: str) -> None:
        self.lines.append(text)

    def write(self, path: Path) -> None:
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


def _fmt(x: float, nd: int = 2) -> str:
    return "--" if not np.isfinite(x) else f"{x:.{nd}f}"


def _alpha(name: str) -> str:
    """LaTeX macro names: letters only."""
    table = str.maketrans("0123456789_", "ABCDEFGHIJ ")
    return "".join(name.translate(table).title().split())


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------


def _plt() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 9,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "figure.dpi": 150,
            "savefig.bbox": "tight",
            "font.family": "serif",
            "mathtext.fontset": "cm",
        }
    )
    return plt


def _crop(
    img: npt.NDArray[Any], cx: float, cy: float, half: int
) -> tuple[npt.NDArray[Any], int, int]:
    h, w = img.shape[:2]
    x0 = int(np.clip(round(cx) - half, 0, w - 2 * half))
    y0 = int(np.clip(round(cy) - half, 0, h - 2 * half))
    return img[y0 : y0 + 2 * half, x0 : x0 + 2 * half], x0, y0


def fig_cameras(scene: Scene, views: list[View], dets: dict[str, Any]) -> None:
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.25))
    for ax, view in zip(axes, views):
        ax.imshow(view.capture.frame.image)
        best = dets[view.camera_id]["best"]
        if best is not None:
            cx, cy = best["centre"]
            ax.add_patch(
                plt.Rectangle(
                    (cx - 40, cy - 40),
                    80,
                    80,
                    fill=False,
                    ec="red",
                    lw=1.0,
                )
            )
        ax.set_title(view.camera_id)
        ax.set_xticks([0, 320, 640, 960, 1279])
        ax.set_yticks([0, 360, 719])
        ax.tick_params(labelsize=6)
        ax.set_xlabel("$u$ [px]", fontsize=7)
        ax.set_ylabel("$v$ [px]", fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"cameras_{scene.robot}.png")
    plt.close(fig)


def fig_overview(scene: Scene, est: Array) -> None:
    import mujoco

    from fret.control.pick_place_vision import set_cv_ball_ghost

    plt = _plt()
    set_cv_ball_ghost(scene.model, scene.data, est)
    cam = mujoco.MjvCamera()
    cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
    cam.lookat[:] = (0.5 * est[0], 0.5 * est[1], 0.12)
    cam.distance = 1.25 if scene.robot == "omx" else 1.9
    cam.azimuth = 145.0
    cam.elevation = -32.0
    with mujoco.Renderer(scene.model, height=540, width=960) as r:
        r.update_scene(scene.data, camera=cam)
        img = r.render()
    fig, ax = plt.subplots(figsize=(4.8, 2.7))
    ax.imshow(img)
    ax.axis("off")
    fig.savefig(FIG_DIR / f"overview_{scene.robot}.png")
    plt.close(fig)
    ghost = mujoco.mj_name2id(
        scene.model, mujoco.mjtObj.mjOBJ_GEOM, "cv_ball_ghost"
    )
    scene.model.geom_rgba[ghost][3] = 0.0


def fig_mask_stages(
    scene: Scene,
    view: View,
    stages: dict[str, Any],
    det: dict[str, Any],
    gt: Array,
) -> None:
    plt = _plt()
    best = det["best"]
    cx, cy = best["centre"]
    half = 28
    rgb, x0, y0 = _crop(view.capture.frame.image, cx, cy, half)
    hue, _, _ = _crop(stages["hsv"][..., 0], cx, cy, half)
    raw, _, _ = _crop(stages["raw"], cx, cy, half)
    opened, _, _ = _crop(stages["opened"], cx, cy, half)
    closed, _, _ = _crop(stages["closed"], cx, cy, half)
    fig, axes = plt.subplots(1, 5, figsize=(7.4, 1.75))
    panels = [
        (rgb, None, "(a) RGB"),
        (hue, "twilight", "(b) $H$ channel"),
        (raw, "gray", "(c) $M_0$ = inRange"),
        (opened, "gray", "(d) opening"),
        (closed, "gray", "(e) closing"),
    ]
    for ax, (img, cmap, title) in zip(axes, panels):
        if cmap is None:
            ax.imshow(img, interpolation="nearest")
        elif cmap == "twilight":
            ax.imshow(
                img, cmap=cmap, vmin=0, vmax=179, interpolation="nearest"
            )
        else:
            ax.imshow(
                img, cmap=cmap, vmin=0, vmax=255, interpolation="nearest"
            )
        ax.set_title(title)
        ax.set_xticks([])
        ax.set_yticks([])
    cont = best["contour"].reshape(-1, 2).astype(float)
    ax = axes[4]
    ax.plot(
        np.r_[cont[:, 0], cont[0, 0]] - x0,
        np.r_[cont[:, 1], cont[0, 1]] - y0,
        "-",
        color="tab:orange",
        lw=0.9,
    )
    circ = plt.Circle(
        (cx - x0, cy - y0), best["radius"], fill=False, ec="tab:red", lw=0.9
    )
    ax.add_patch(circ)
    u_gt, v_gt, _ = project(
        gt, view.capture.intrinsics, view.capture.extrinsics
    )
    ax.plot(cx - x0, cy - y0, "+", color="tab:red", ms=7, mew=1.0)
    ax.plot(u_gt - x0, v_gt - y0, "x", color="tab:cyan", ms=5, mew=1.0)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"mask_stages_{scene.robot}.png")
    plt.close(fig)


def fig_hsv_scatter(
    scene: Scene, views: list[View], det_cfg: Any
) -> dict[str, float]:
    """Ball vs background pixels in (H, S) with the threshold box."""
    import cv2

    plt = _plt()
    ball_hsv = []
    bg_hsv = []
    for v in views:
        bgr = cv2.cvtColor(v.capture.frame.image, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        seg = v.ball_seg.reshape(-1)
        ball_hsv.append(hsv[seg])
        rng = np.random.default_rng(0)
        idx = rng.choice(np.flatnonzero(~seg), size=40000, replace=False)
        bg_hsv.append(hsv[idx])
    ball = np.concatenate(ball_hsv).astype(float)
    bg = np.concatenate(bg_hsv).astype(float)
    lo = np.asarray(det_cfg.hsv_lower, dtype=float)
    hi = np.asarray(det_cfg.hsv_upper, dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
    for ax, (i, j, li, lj) in zip(
        axes, [(0, 1, "$H$", "$S$"), (0, 2, "$H$", "$V$")]
    ):
        ax.scatter(
            bg[:, i],
            bg[:, j],
            s=1,
            c="0.6",
            alpha=0.25,
            lw=0,
            label="background (sample)",
            rasterized=True,
        )
        ax.scatter(
            ball[:, i],
            ball[:, j],
            s=2,
            c="tab:olive",
            alpha=0.6,
            lw=0,
            label="ball (MuJoCo segmentation)",
            rasterized=True,
        )
        ax.add_patch(
            plt.Rectangle(
                (lo[i], lo[j]),
                hi[i] - lo[i],
                hi[j] - lo[j],
                fill=False,
                ec="tab:red",
                lw=1.0,
                label="box $[\\ell, h]$",
            )
        )
        ax.set_xlim(0, 180)
        ax.set_ylim(0, 256)
        ax.set_xlabel(li)
        ax.set_ylabel(lj)
    axes[0].legend(loc="upper right", markerscale=4, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"hsv_{scene.robot}.png", dpi=200)
    plt.close(fig)
    inside = np.all((ball >= lo) & (ball <= hi), axis=1)
    bg_inside = np.all((bg >= lo) & (bg <= hi), axis=1)
    return {
        "ball_frac_inside": float(np.mean(inside)),
        "bg_frac_inside": float(np.mean(bg_inside)),
        "ball_h_median": float(np.median(ball[:, 0])),
        "ball_s_median": float(np.median(ball[:, 1])),
        "ball_v_median": float(np.median(ball[:, 2])),
        "ball_h_p01": float(np.percentile(ball[:, 0], 1)),
        "ball_h_p99": float(np.percentile(ball[:, 0], 99)),
        "ball_v_p01": float(np.percentile(ball[:, 2], 1)),
    }


def fig_geometry(
    scene: Scene,
    views: list[View],
    points: dict[str, Array],
    fused: Array,
    gt: Array,
) -> None:
    plt = _plt()
    fig = plt.figure(figsize=(7.2, 3.3))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    xs = np.linspace(-0.2, 0.4, 2)
    ys = np.linspace(-0.4, 0.4, 2)
    xx, yy = np.meshgrid(xs, ys)
    plane_z = scene.cfg.lifter.table_z_m + scene.cfg.lifter.ball_radius_m
    ax.plot_surface(
        xx, yy, np.full_like(xx, plane_z), alpha=0.15, color="tab:blue"
    )
    colors = {"gate_cam_left": "tab:green", "gate_cam_right": "tab:purple"}
    for v in views:
        ex = v.capture.extrinsics.t_world_cam
        o = ex[:3, 3]
        rot = ex[:3, :3]
        for k, c in zip(range(3), ("r", "g", "b")):
            ax.quiver(*o, *(0.08 * rot[:, k]), color=c, lw=0.8)
        if v.camera_id in points:
            p = points[v.camera_id]
            ax.plot(
                *np.c_[o, p],
                color=colors[v.camera_id],
                lw=1.0,
                label=f"{CAM_LABEL[v.camera_id]} ray",
            )
        ax.text(
            *o, f"  $O_{{{CAM_LABEL[v.camera_id][0].upper()}}}$", fontsize=7
        )
    ax.scatter(*gt, color="gold", s=20, edgecolor="k", label="ball (true)")
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_zlabel("$z$ [m]")
    ax.view_init(elev=24, azim=-128)
    ax.legend(loc="upper left", fontsize=6)
    ax.set_box_aspect((0.6, 0.8, 0.75))
    ax2 = fig.add_subplot(1, 2, 2)
    for cam, p in points.items():
        d = (p[:2] - gt[:2]) * 1000
        ax2.plot(
            d[0],
            d[1],
            "o",
            color=colors[cam],
            label=f"$\\hat p_{{{CAM_LABEL[cam][0].upper()}}}$ (mono)",
        )
    d = (fused[:2] - gt[:2]) * 1000
    ax2.plot(d[0], d[1], "*", color="tab:red", ms=10, label="fused $\\hat p$")
    ax2.plot(0, 0, "+", color="k", ms=12, label="truth ($p^\\star$)")
    ax2.set_xlabel("$\\Delta x$ [mm]")
    ax2.set_ylabel("$\\Delta y$ [mm]")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=7)
    ax2.set_title("error on the table plane")
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"geometry_{scene.robot}.png")
    plt.close(fig)


def fig_occlusion(
    scene: Scene,
    view: View,
    stages: dict[str, Any],
    det: dict[str, Any],
    gt: Array,
) -> dict[str, float]:
    plt = _plt()
    best = det["best"]
    u_gt, v_gt, _ = project(
        gt, view.capture.intrinsics, view.capture.extrinsics
    )
    half = 32
    rgb, x0, y0 = _crop(view.capture.frame.image, u_gt, v_gt, half)
    closed, _, _ = _crop(stages["closed"], u_gt, v_gt, half)
    seg, _, _ = _crop(view.ball_seg.astype(np.uint8) * 255, u_gt, v_gt, half)
    ell = sphere_ellipse(
        gt,
        scene.cfg.lifter.ball_radius_m,
        view.capture.intrinsics,
        view.capture.extrinsics,
    )
    fig, axes = plt.subplots(1, 3, figsize=(6.0, 2.1))
    for ax, img, title in zip(
        axes,
        (rgb, seg, closed),
        ("(a) RGB", "(b) visible ball pixels", "(c) mask + circle"),
    ):
        ax.imshow(
            img,
            cmap=None if img.ndim == 3 else "gray",
            interpolation="nearest",
        )
        ax.set_title(title)
        ax.set_xticks([])
        ax.set_yticks([])
        full = plt.Circle(
            (ell["u_centre"] - x0, ell["v_centre"] - y0),
            ell["a_px"],
            fill=False,
            ec="tab:cyan",
            lw=0.8,
            ls="--",
        )
        ax.add_patch(full)
    if best is not None:
        cx, cy = best["centre"]
        axes[2].add_patch(
            plt.Circle(
                (cx - x0, cy - y0),
                best["radius"],
                fill=False,
                ec="tab:red",
                lw=1.0,
            )
        )
        axes[2].plot(cx - x0, cy - y0, "+", color="tab:red", ms=7)
    axes[2].plot(u_gt - x0, v_gt - y0, "x", color="tab:cyan", ms=6)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"occlusion_{scene.robot}.png")
    plt.close(fig)
    vis = float(view.ball_seg.sum() / ell["area_px"])
    err = (
        float(np.hypot(best["centre"][0] - u_gt, best["centre"][1] - v_gt))
        if best is not None
        else float("nan")
    )
    return {
        "visible_frac": vis,
        "centre_err_px": err,
        "circ": float(best["circularity"]) if best else float("nan"),
        "radius_px": float(best["radius"]) if best else float("nan"),
        "a_px": ell["a_px"],
    }


def _digital_disk(
    shape: tuple[int, int], cx: float, cy: float, a: float, b: float
) -> npt.NDArray[np.uint8]:
    yy, xx = np.mgrid[0 : shape[0], 0 : shape[1]]
    inside = ((xx - cx) / a) ** 2 + ((yy - cy) / b) ** 2 <= 1.0
    return (inside * 255).astype(np.uint8)


def fig_circularity() -> dict[str, float]:
    import cv2

    plt = _plt()
    rng = np.random.default_rng(7)
    radii = np.array([3, 4, 5, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60])
    means = []
    stds = []
    for r in radii:
        vals = []
        for _ in range(16):
            n = int(2 * r + 12)
            cx, cy = n / 2 + rng.uniform(-0.5, 0.5, size=2)
            m = _digital_disk((n, n), cx, cy, r, r)
            c, _ = cv2.findContours(
                m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            c = max(c, key=cv2.contourArea)
            area = cv2.contourArea(c)
            per = cv2.arcLength(c, True)
            vals.append(4 * np.pi * area / per**2)
        means.append(np.mean(vals))
        stds.append(np.std(vals))
    means_a = np.asarray(means)

    # Half-plane occlusion of a disk (R = 30 px): MEC centre error vs
    # analytic prediction; centroid for comparison.
    r0 = 30.0
    n = 100
    cx = cy = n / 2 + 0.3
    fracs = []
    mec_err = []
    cen_err = []
    circs = []
    analytic = []
    for d in np.linspace(-0.95 * r0, 0.95 * r0, 39):
        m = _digital_disk((n, n), cx, cy, r0, r0)
        xx = np.arange(n)[None, :].repeat(n, axis=0)
        m[xx > cx + d] = 0
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        (ux, uy), _rad = cv2.minEnclosingCircle(c)
        mom = cv2.moments(m, binaryImage=True)
        gx, gy = mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]
        frac = float((m > 0).sum() / (np.pi * r0 * r0))
        fracs.append(frac)
        mec_err.append(np.hypot(ux - cx, uy - cy) / r0)
        cen_err.append(np.hypot(gx - cx, gy - cy) / r0)
        area = cv2.contourArea(c)
        per = cv2.arcLength(c, True)
        circs.append(4 * np.pi * area / per**2)
        analytic.append(abs(d) / r0 if d < 0 else 0.0)

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.6))
    ax = axes[0]
    ax.errorbar(
        radii,
        means_a,
        yerr=stds,
        fmt="o-",
        ms=3,
        lw=1,
        capsize=2,
        label="disco digital (16 offsets)",
    )
    ax.axhline(1.0, color="k", lw=0.6, ls=":")
    ax.axhline(
        0.45,
        color="tab:red",
        lw=0.8,
        ls="--",
        label="$c_{\\min}=0{,}45$ (gate)",
    )
    ax.set_xscale("log")
    ax.set_xticks([3, 5, 10, 20, 30, 60])
    ax.set_xticks([], minor=True)
    ax.set_xticklabels(["3", "5", "10", "20", "30", "60"])
    ax.set_xlabel("radius $R$ [px]")
    ax.set_ylabel("circularity $c$")
    ax.set_ylim(0.4, 1.05)
    ax.legend(loc="lower right")
    ax.grid(alpha=0.3, which="both")
    ax = axes[1]
    order = np.argsort(fracs)
    f = np.asarray(fracs)[order]
    ax.plot(
        f, np.asarray(mec_err)[order], "o", ms=3, label="MEC centre (measured)"
    )
    ax.plot(
        f, np.asarray(analytic)[order], "-", lw=1, label="MEC centre (theory)"
    )
    ax.plot(
        f,
        np.asarray(cen_err)[order],
        "s",
        ms=2.5,
        label="centroid (moments)",
    )
    ax.plot(
        f,
        np.asarray(circs)[order],
        "-.",
        lw=1,
        color="0.4",
        label="circularity $c$",
    )
    ax.axvline(0.5, color="k", lw=0.6, ls=":")
    ax.set_xlabel("visible fraction of the disk")
    ax.set_ylabel("centre error $/R$  (or $c$)")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "circularity.png")
    plt.close(fig)
    idx5 = int(np.where(radii == 5)[0][0])
    idx10 = int(np.where(radii == 10)[0][0])
    idx30 = int(np.where(radii == 30)[0][0])
    above = [e for fr, e in zip(fracs, mec_err) if fr > 0.52]
    return {
        "circ_r5": float(means_a[idx5]),
        "circ_r10": float(means_a[idx10]),
        "circ_r30": float(means_a[idx30]),
        "circ_r60": float(means_a[-1]),
        "mec_err_above_half_max": float(max(above)),
    }


def fig_perspective_bias(scenes: dict[str, Scene]) -> dict[str, float]:
    plt = _plt()
    fig, ax = plt.subplots(figsize=(4.6, 2.7))
    out: dict[str, float] = {}
    for robot, scene in scenes.items():
        intr = scene.cfg.vision.intrinsics_for(CAMS[0])
        extr = scene.cfg.vision.extrinsics_for(CAMS[0])
        r = scene.cfg.lifter.ball_radius_m
        tan_max = np.hypot(intr.width / 2, intr.height / 2) / intr.fx
        alpha = np.linspace(0, np.arctan(tan_max), 200)
        # Typical camera-ball distance at the default pick.
        x, y = ROBOTS[robot]["sweep"][0][1:]
        dist = np.linalg.norm(
            np.array([x, y, ROBOTS[robot]["ball_z"]]) - extr.t_world_cam[:3, 3]
        )
        beta = np.arcsin(r / dist)
        bias = (
            intr.fx
            * np.tan(alpha)
            * np.sin(beta) ** 2
            / (np.cos(beta) ** 2 - np.sin(alpha) ** 2)
        )
        ax.plot(
            np.degrees(alpha),
            bias,
            lw=1.2,
            label=f"{robot.upper().replace('OM', 'OM-')}: $r={r*1000:.0f}$ mm, $D={dist:.2f}$ m",
        )
        # Numerical check: project points of the tangency circle.
        checks = []
        for a in np.linspace(0.05, 0.95 * np.arctan(tan_max), 7):
            pc = dist * np.array([np.sin(a), 0.0, np.cos(a)])
            ch = pc / dist
            # tangency circle: centre pc*(1 - sin^2 beta), radius r cos beta
            centre = pc * (1 - np.sin(beta) ** 2)
            rad = r * np.cos(beta)
            e1 = np.array([np.cos(a), 0.0, -np.sin(a)])
            e2 = np.array([0.0, 1.0, 0.0])
            th = np.linspace(0, 2 * np.pi, 20001)
            pts = centre[None, :] + rad * (
                np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2
            )
            xn = pts[:, 0] / pts[:, 2]
            centre_x = 0.5 * (xn.max() + xn.min())
            checks.append((np.degrees(a), intr.fx * (centre_x - np.tan(a))))
            del ch
        c = np.asarray(checks)
        ax.plot(
            c[:, 0],
            c[:, 1],
            "o",
            ms=3.5,
            mfc="none",
            color=ax.lines[-1].get_color(),
        )
        out[f"{robot}_bias_max_px"] = float(bias[-1])
        out[f"{robot}_alpha_max_deg"] = float(np.degrees(alpha[-1]))
        out[f"{robot}_check_maxdiff_px"] = float(
            np.max(
                np.abs(
                    c[:, 1]
                    - intr.fx
                    * np.tan(np.radians(c[:, 0]))
                    * np.sin(beta) ** 2
                    / (np.cos(beta) ** 2 - np.sin(np.radians(c[:, 0])) ** 2)
                )
            )
        )
    ax.set_xlabel("off-axis angle $\\alpha$ [deg]")
    ax.set_ylabel("bias $f\\,\\delta$ [px]")
    ax.grid(alpha=0.3)
    ax.legend(
        fontsize=7,
        title="line: closed form; o: numerical projection",
        title_fontsize=7,
    )
    fig.tight_layout()
    fig.savefig(FIG_DIR / "perspective_bias.png")
    plt.close(fig)
    return out


def fig_sensitivity(scene: Scene, sweep_xy: list[tuple[float, float]]) -> None:
    plt = _plt()
    vision = scene.cfg.vision
    plane_z = scene.cfg.lifter.table_z_m + scene.cfg.lifter.ball_radius_m
    gx = np.linspace(-0.05, 0.6 if scene.robot == "omy" else 0.42, 120)
    gy = np.linspace(-0.5, 0.5, 160)
    maps = {}
    jac: dict[str, npt.NDArray[np.float64]] = {}
    for cam in CAMS:
        intr = vision.intrinsics_for(cam)
        extr = vision.extrinsics_for(cam)
        sig = np.full((gy.size, gx.size), np.nan)
        jj = np.full((gy.size, gx.size, 2, 2), np.nan)
        for i, y in enumerate(gy):
            for j, x in enumerate(gx):
                u, v, z = project(np.array([x, y, plane_z]), intr, extr)
                if z <= 0 or not (
                    0 <= u < intr.width and 0 <= v < intr.height
                ):
                    continue
                jm = lift_jacobian(u, v, intr, extr, plane_z)
                jj[i, j] = jm
                sig[i, j] = np.linalg.svd(jm, compute_uv=False)[0] * 1000
        maps[cam] = sig
        jac[cam] = jj
    jl, jr = jac[CAMS[0]], jac[CAMS[1]]
    fused = np.full((gy.size, gx.size), np.nan)
    both = ~np.isnan(jl[..., 0, 0]) & ~np.isnan(jr[..., 0, 0])
    for i, j in zip(*np.nonzero(both)):
        cov = 0.25 * (jl[i, j] @ jl[i, j].T + jr[i, j] @ jr[i, j].T)
        fused[i, j] = np.sqrt(np.linalg.eigvalsh(cov)[-1]) * 1000
    fig, axes = plt.subplots(1, 3, figsize=(7.4, 2.9), sharey=True)
    vmax = np.nanpercentile(
        np.r_[maps[CAMS[0]].ravel(), maps[CAMS[1]].ravel()], 98
    )
    titles = (
        "left camera: $\\sigma_{\\max}(J_L)$",
        "right camera: $\\sigma_{\\max}(J_R)$",
        "fused (equal weights)",
    )
    for ax, data, title in zip(
        axes, (maps[CAMS[0]], maps[CAMS[1]], fused), titles
    ):
        im = ax.pcolormesh(
            gx, gy, data, shading="auto", vmin=0, vmax=vmax, cmap="viridis"
        )
        ax.set_title(title)
        ax.set_xlabel("$x$ [m]")
        ax.set_aspect("equal")
        for x, y in sweep_xy:
            ax.plot(x, y, "w.", ms=3)
        ax.plot(0, 0, "r^", ms=4)
    axes[0].set_ylabel("$y$ [m]")
    cb = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.02)
    cb.set_label("mm per pixel")
    fig.savefig(FIG_DIR / f"sensitivity_{scene.robot}.png")
    plt.close(fig)


def fig_error_map(scene: Scene, rows: list[dict[str, Any]]) -> None:
    import mujoco

    plt = _plt()
    fig, ax = plt.subplots(figsize=(4.4, 4.0))
    # Place bin footprint and robot base.
    for gid in range(scene.model.ngeom):
        name = (
            mujoco.mj_id2name(scene.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        )
        if name.startswith("place_bin") and scene.model.geom_type[gid] == int(
            mujoco.mjtGeom.mjGEOM_BOX
        ):
            p = scene.data.geom_xpos[gid]
            s = scene.model.geom_size[gid]
            ax.add_patch(
                plt.Rectangle(
                    (p[0] - s[0], p[1] - s[1]),
                    2 * s[0],
                    2 * s[1],
                    fc="0.85",
                    ec="0.5",
                    lw=0.5,
                )
            )
    ax.plot(0, 0, "k^", ms=6, label="arm base")
    errs = np.array([r["xy_err_mm"] for r in rows])
    finite = np.isfinite(errs)
    xs = np.array([r["ball"][0] for r in rows])
    ys = np.array([r["ball"][1] for r in rows])
    nv = np.array([r["views"] for r in rows])
    vmax = max(
        5.0, float(np.nanpercentile(errs[finite], 95)) if finite.any() else 5
    )
    sc = None
    for k, mk, lab in ((2, "o", "2 views"), (1, "s", "1 view")):
        sel = finite & (nv == k)
        if sel.any():
            sc = ax.scatter(
                xs[sel],
                ys[sel],
                c=errs[sel],
                cmap="plasma",
                vmin=0,
                vmax=vmax,
                marker=mk,
                s=22,
                ec="k",
                lw=0.3,
                label=lab,
            )
    miss = ~finite
    ax.scatter(
        xs[miss],
        ys[miss],
        marker="x",
        c="tab:red",
        s=14,
        lw=0.8,
        label="no detection",
    )
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=6.5, loc="upper left", bbox_to_anchor=(1.25, 1.0))
    if sc is not None:
        cb = fig.colorbar(sc, ax=ax, shrink=0.75, pad=0.03)
        cb.set_label("XY error [mm]")
    fig.savefig(FIG_DIR / f"error_map_{scene.robot}.png")
    plt.close(fig)


# --------------------------------------------------------------------------
# Analyses
# --------------------------------------------------------------------------


def analyse_pose(
    scene: Scene, x: float, y: float, *, with_seg: bool = True
) -> dict[str, Any]:
    from fret.vision.types import CameraFrame

    gt = set_ball(scene, x, y)
    views = render_views(scene, with_seg=with_seg)
    frames: list[CameraFrame] = [v.capture.frame for v in views]
    obs = scene.pipeline.process(frames)
    det_list = scene.detector.detect(frames)
    plane_z = scene.cfg.lifter.table_z_m + scene.cfg.lifter.ball_radius_m
    per: dict[str, Any] = {}
    stages_by: dict[str, Any] = {}
    dets_by: dict[str, Any] = {}
    for v in views:
        st = mask_stages(v.capture.frame.image, scene.cfg.detector)
        ref = scene.detector.mask_for_frame(v.capture.frame)
        assert np.array_equal(st["closed"], ref), "mask mismatch vs detector"
        cs = contour_stats(st["closed"], scene.cfg.detector)
        stages_by[v.camera_id] = st
        dets_by[v.camera_id] = cs
        intr, extr = v.capture.intrinsics, v.capture.extrinsics
        u_gt, v_gt, depth = project(gt, intr, extr)
        ell = sphere_ellipse(gt, scene.cfg.lifter.ball_radius_m, intr, extr)
        entry: dict[str, Any] = {
            "u_gt": u_gt,
            "v_gt": v_gt,
            "depth": depth,
            "ellipse": ell,
            "seg_px": int(v.ball_seg.sum()),
            "visible_frac": float(v.ball_seg.sum() / ell["area_px"]),
        }
        best = cs["best"]
        if best is not None:
            match = [d for d in det_list if d.camera_id == v.camera_id]
            assert match and np.allclose(match[0].centre_px, best["centre"])
            u, vv = best["centre"]
            o, d = ray_world(u, vv, intr, extr)
            t = (plane_z - o[2]) / d[2]
            p = o + t * d
            pts = best["contour"].reshape(-1, 2).astype(np.float64)
            mec_gap = float(
                np.max(np.hypot(pts[:, 0] - u, pts[:, 1] - vv))
                - best["radius"]
            )
            mask_bool = st["closed"] > 0
            inter = int((mask_bool & v.ball_seg).sum())
            union = int((mask_bool | v.ball_seg).sum())
            entry.update(
                {
                    "u": u,
                    "v": vv,
                    "radius_px": best["radius"],
                    "area_px": best["area"],
                    "perimeter_px": best["perimeter"],
                    "circ": best["circularity"],
                    "n_contours": len(cs["all"]),
                    "centroid": best["centroid"],
                    "d_cam": np.array(
                        [
                            (u - intr.cx) / intr.fx,
                            (vv - intr.cy) / intr.fy,
                            1.0,
                        ]
                    ),
                    "origin": o,
                    "d_world": d,
                    "t": float(t),
                    "point": p,
                    "centre_err_px": float(np.hypot(u - u_gt, vv - v_gt)),
                    "iou": inter / union if union else float("nan"),
                    "mec_gap": mec_gap,
                    "jac": lift_jacobian(u, vv, intr, extr, plane_z),
                    "tan_incidence": float(np.linalg.norm(d[:2]) / abs(d[2])),
                }
            )
        per[v.camera_id] = entry
    hits = [c for c in CAMS if "point" in per[c]]
    fused = None
    if hits:
        w = np.array([max(per[c]["circ"], 1e-3) for c in hits])
        w = w / w.sum()
        fused = np.sum(
            [wi * per[c]["point"] for wi, c in zip(w, hits)], axis=0
        )
        fused[2] = plane_z
        assert obs is not None
        assert np.allclose(fused, obs.position_world, atol=1e-12), (
            fused,
            obs.position_world,
        )
        for wi, c in zip(w, hits):
            per[c]["weight"] = float(wi)
    else:
        assert obs is None
    return {
        "gt": gt,
        "views": views,
        "per": per,
        "stages": stages_by,
        "dets": dets_by,
        "fused": fused,
        "obs": obs,
        "hits": hits,
        "xy_err_mm": (
            float(np.linalg.norm(fused[:2] - gt[:2]) * 1000)
            if fused is not None
            else float("nan")
        ),
        "z_err_mm": (
            float(abs(fused[2] - gt[2]) * 1000)
            if fused is not None
            else float("nan")
        ),
    }


def monte_carlo(
    scene: Scene, res: dict[str, Any], sigma_px: float, n: int
) -> dict[str, float]:
    """Propagate isotropic pixel noise through lift + fuse; compare to J."""
    rng = np.random.default_rng(42)
    plane_z = scene.cfg.lifter.table_z_m + scene.cfg.lifter.ball_radius_m
    hits = res["hits"]
    w = np.array([res["per"][c]["weight"] for c in hits])
    samples = np.zeros((n, 2))
    for k in range(n):
        acc = np.zeros(3)
        for wi, c in zip(w, hits):
            e = res["per"][c]
            intr = scene.cfg.vision.intrinsics_for(c)
            extr = scene.cfg.vision.extrinsics_for(c)
            du, dv = rng.normal(0.0, sigma_px, size=2)
            p = lift_pixel(e["u"] + du, e["v"] + dv, intr, extr, plane_z)
            assert p is not None
            acc += wi * p
        samples[k] = acc[:2]
    emp = np.cov(samples.T)
    pred = (
        sum(
            wi**2 * res["per"][c]["jac"] @ res["per"][c]["jac"].T
            for wi, c in zip(w, hits)
        )
        * sigma_px**2
    )
    ev_emp = np.sqrt(np.linalg.eigvalsh(emp)) * 1000
    ev_pred = np.sqrt(np.linalg.eigvalsh(pred)) * 1000
    return {
        "emp_min_mm": float(ev_emp[0]),
        "emp_max_mm": float(ev_emp[1]),
        "pred_min_mm": float(ev_pred[0]),
        "pred_max_mm": float(ev_pred[1]),
        "frob_rel": float(np.linalg.norm(emp - pred) / np.linalg.norm(pred)),
    }


def jacobian_fd_check(scene: Scene, res: dict[str, Any]) -> float:
    plane_z = scene.cfg.lifter.table_z_m + scene.cfg.lifter.ball_radius_m
    worst = 0.0
    for c in res["hits"]:
        e = res["per"][c]
        intr = scene.cfg.vision.intrinsics_for(c)
        extr = scene.cfg.vision.extrinsics_for(c)
        h = 1e-4
        cols = []
        for du, dv in ((h, 0.0), (0.0, h)):
            p1 = lift_pixel(e["u"] + du, e["v"] + dv, intr, extr, plane_z)
            p0 = lift_pixel(e["u"] - du, e["v"] - dv, intr, extr, plane_z)
            assert p0 is not None and p1 is not None
            cols.append((p1[:2] - p0[:2]) / (2 * h))
        fd = np.column_stack(cols)
        worst = max(worst, float(np.max(np.abs(fd - e["jac"]))))
    return worst


def calib_check(scene: Scene) -> dict[str, float]:
    out = {"orth": 0.0, "det": 0.0, "ext": 0.0, "fx": 0.0}
    for c in CAMS:
        live_i = scene.adapters[c].live_intrinsics()
        live_e = scene.adapters[c].live_extrinsics()
        yi = scene.cfg.vision.intrinsics_for(c)
        ye = scene.cfg.vision.extrinsics_for(c)
        rot = ye.t_world_cam[:3, :3]
        out["orth"] = max(
            out["orth"], float(np.abs(rot.T @ rot - np.eye(3)).max())
        )
        out["det"] = max(out["det"], float(abs(np.linalg.det(rot) - 1.0)))
        out["ext"] = max(
            out["ext"],
            float(np.abs(live_e.t_world_cam - ye.t_world_cam).max()),
        )
        out["fx"] = max(
            out["fx"], abs(live_i.fx - yi.fx), abs(live_i.cx - yi.cx)
        )
    return out


def ik_residuals(scene: Scene, ball: Array) -> dict[str, float]:
    import mujoco

    from fret.control.pick_place_sim import waypoints_from_scenario
    from fret.control.pick_place_vision import (
        _FLOOR_GRASP_PAD_LIFT_M,
        refresh_pick_waypoints_omx,
        refresh_pick_waypoints_omy,
    )

    base = waypoints_from_scenario(REPO / scene.spec["scenario_yml"])
    data = mujoco.MjData(scene.model)
    data.qpos[:] = scene.data.qpos
    mujoco.mj_forward(scene.model, data)
    if scene.robot == "omx":
        wp = refresh_pick_waypoints_omx(base, ball, scene.model, data)
        joints = ("Joint1", "Joint2", "Joint3", "Joint4")
        grip = ("Gripper", 0.019, 0.006)
        hover, lift = 0.08, 0.12
    else:
        from fret.control.omy_pad_mid_ik import (
            OMY_ARM_JOINTS,
            OMY_GRIPPER_PINCH,
        )

        wp = refresh_pick_waypoints_omy(base, ball, scene.model, data)
        joints = OMY_ARM_JOINTS
        grip = ("rh_r1", 0.0, OMY_GRIPPER_PINCH)
        hover, lift = 0.10, 0.14
    m = scene.model
    pr = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pad_right")
    pl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pad_left")
    gadr = m.jnt_qposadr[
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, grip[0])
    ]
    out = {}
    targets = {
        "hover": (wp.pick_hover, ball + [0, 0, hover], grip[1]),
        "grasp": (
            wp.pick_grasp,
            ball + [0, 0, _FLOOR_GRASP_PAD_LIFT_M],
            grip[2],
        ),
        "lift": (wp.lift_hover, ball + [0, 0, lift], grip[2]),
    }
    for key, (q, tgt, gv) in targets.items():
        assert q is not None
        for i, name in enumerate(joints):
            adr = m.jnt_qposadr[
                mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            data.qpos[adr] = q[i]
        data.qpos[gadr] = gv
        mujoco.mj_forward(m, data)
        mid = 0.5 * (data.geom_xpos[pr] + data.geom_xpos[pl])
        out[f"{key}_mm"] = float(np.linalg.norm(mid - tgt) * 1000)
        out[f"{key}_q"] = [float(v) for v in q]  # type: ignore[assignment]
    return out


def sweep_table(rows: list[dict[str, Any]]) -> tuple[str, str]:
    """Return the sweep table body as (LaTeX rows, Markdown rows)."""
    lines = []
    md = []
    for r in rows:
        per = r["per"]
        vis = " / ".join(_fmt(per[c]["visible_frac"] * 100, 0) for c in CAMS)
        circ = " / ".join(
            _fmt(per[c]["circ"], 2) if "circ" in per[c] else "--" for c in CAMS
        )
        cerr = " / ".join(
            (
                _fmt(per[c]["centre_err_px"], 2)
                if "centre_err_px" in per[c]
                else "--"
            )
            for c in CAMS
        )
        lines.append(
            f"\\texttt{{{r['pose'].replace('_', chr(92) + '_')}}} & "
            f"$({_fmt(r['xy'][0])}, {_fmt(r['xy'][1])})$ & {len(r['hits'])} & "
            f"{vis} & {circ} & {cerr} & {_fmt(r['xy_err_mm'], 2)} \\\\"
        )
        md.append(
            f"| `{r['pose']}` | ({_fmt(r['xy'][0])}, {_fmt(r['xy'][1])}) | "
            f"{len(r['hits'])} | {vis} | {circ} | {cerr} | "
            f"{_fmt(r['xy_err_mm'], 2)} |"
        )
    return "\n".join(lines), "\n".join(md)


def render_markdown(mac: Macros) -> None:
    """Fill ``{{Name}}`` placeholders of the Markdown template."""
    template = (DOC_DIR / "vision_algorithm.template.md").read_text(
        encoding="utf-8"
    )
    out = re.sub(r"\{\{(\w+)\}\}", lambda m: mac.text[m.group(1)], template)
    banner = (
        "<!-- Generated by scripts/vision_algorithm_figures.py from "
        "vision_algorithm.template.md -- edit the template. -->\n"
    )
    (DOC_DIR / "vision_algorithm.md").write_text(
        banner + out, encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-grid",
        action="store_true",
        help="Skip the dense XY error map (faster)",
    )
    parser.add_argument("--mc-samples", type=int, default=20000)
    args = parser.parse_args()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    mac = Macros()
    metrics: dict[str, Any] = {}

    scenes = {r: load_scene(r) for r in ROBOTS}
    try:
        metrics["circularity"] = fig_circularity()
        for k, v in metrics["circularity"].items():
            mac.add(_alpha(f"circ_{k}"), v)
        metrics["bias"] = fig_perspective_bias(scenes)
        for k, v in metrics["bias"].items():
            mac.add(
                _alpha(f"bias_{k}"), v, "{:.3f}" if "px" in k else "{:.1f}"
            )

        for robot, scene in scenes.items():
            pre = robot
            spec = scene.spec
            cal = calib_check(scene)
            metrics[f"{robot}_calib"] = cal
            for k, v in cal.items():
                mac.add(_alpha(f"{pre}_cal_{k}"), v, "{:.1e}")
            intr = scene.cfg.vision.intrinsics_for(CAMS[0])
            mac.add(_alpha(f"{pre}_fx"), intr.fx, "{:.4f}")
            mac.add(
                _alpha(f"{pre}_fovy"),
                float(scene.model.cam_fovy[scene.adapters[CAMS[0]]._cam_id]),
                "{:.0f}",
            )
            mac.add(
                _alpha(f"{pre}_ballr"),
                scene.cfg.lifter.ball_radius_m * 1000,
                "{:.0f}",
            )

            # Worked example at the default pick pose.
            name, x, y = spec["sweep"][0]
            res = analyse_pose(scene, x, y)
            fig_cameras(scene, res["views"], res["dets"])
            fig_mask_stages(
                scene,
                res["views"][0],
                res["stages"][CAMS[0]],
                res["dets"][CAMS[0]],
                res["gt"],
            )
            hsv_stats = fig_hsv_scatter(
                scene, res["views"], scene.cfg.detector
            )
            metrics[f"{robot}_hsv"] = hsv_stats
            for k, v in hsv_stats.items():
                mac.add(
                    _alpha(f"{pre}_hsv_{k}"),
                    v * 100 if "frac" in k else v,
                    "{:.2f}" if "frac" in k else "{:.0f}",
                )
            fig_geometry(
                scene,
                res["views"],
                {c: res["per"][c]["point"] for c in res["hits"]},
                res["fused"],
                res["gt"],
            )
            ex: dict[str, Any] = {
                "gt": res["gt"].tolist(),
                "fused": res["fused"].tolist(),
            }
            for c in CAMS:
                e = res["per"][c]
                tag = f"{pre}_{CAM_LABEL[c][0]}"
                for key, fmt in (
                    ("u", "{:.2f}"),
                    ("v", "{:.2f}"),
                    ("u_gt", "{:.2f}"),
                    ("v_gt", "{:.2f}"),
                    ("radius_px", "{:.2f}"),
                    ("area_px", "{:.1f}"),
                    ("perimeter_px", "{:.2f}"),
                    ("circ", "{:.4f}"),
                    ("t", "{:.4f}"),
                    ("weight", "{:.4f}"),
                    ("centre_err_px", "{:.2f}"),
                    ("iou", "{:.3f}"),
                    ("depth", "{:.4f}"),
                    ("tan_incidence", "{:.3f}"),
                    ("n_contours", "{:.0f}"),
                ):
                    mac.add(_alpha(f"{tag}_{key}"), float(e[key]), fmt)
                for i, axn in enumerate("xyz"):
                    mac.add(_alpha(f"{tag}_dc_{axn}"), e["d_cam"][i], "{:.5f}")
                    mac.add(
                        _alpha(f"{tag}_dw_{axn}"), e["d_world"][i], "{:.5f}"
                    )
                    mac.add(_alpha(f"{tag}_o_{axn}"), e["origin"][i], "{:.3f}")
                    mac.add(
                        _alpha(f"{tag}_p_{axn}"),
                        e["point"][i] * 1000,
                        "{:.2f}",
                    )
                ell = e["ellipse"]
                mac.add(_alpha(f"{tag}_ell_a"), ell["a_px"], "{:.2f}")
                mac.add(_alpha(f"{tag}_ell_b"), ell["b_px"], "{:.2f}")
                mac.add(_alpha(f"{tag}_ell_bias"), ell["bias_px"], "{:.3f}")
                mac.add(
                    _alpha(f"{tag}_ell_alpha"),
                    np.degrees(ell["alpha"]),
                    "{:.2f}",
                )
                mac.add(_alpha(f"{tag}_ell_dist"), ell["dist"], "{:.4f}")
                mac.add(_alpha(f"{tag}_ell_area"), ell["area_px"], "{:.1f}")
                mac.add(_alpha(f"{tag}_segpx"), e["seg_px"], "{:.0f}")
                jm = e["jac"] * 1000
                for i in range(2):
                    for j in range(2):
                        mac.add(
                            _alpha(f"{tag}_j_{'ab'[i]}{'ab'[j]}"),
                            jm[i, j],
                            "{:.4f}",
                        )
                mac.add(
                    _alpha(f"{tag}_jsig"),
                    np.linalg.svd(jm, compute_uv=False)[0],
                    "{:.3f}",
                )
                ex[c] = {
                    k: (v.tolist() if isinstance(v, np.ndarray) else v)
                    for k, v in e.items()
                    if k not in ("ellipse", "centroid")
                }
            for i, axn in enumerate("xyz"):
                mac.add(
                    _alpha(f"{pre}_gt_{axn}"), res["gt"][i] * 1000, "{:.2f}"
                )
                mac.add(
                    _alpha(f"{pre}_fu_{axn}"), res["fused"][i] * 1000, "{:.2f}"
                )
            mac.add(_alpha(f"{pre}_ex_xyerr"), res["xy_err_mm"], "{:.3f}")
            mac.add(_alpha(f"{pre}_ex_zerr"), res["z_err_mm"], "{:.3f}")
            mac.add(
                _alpha(f"{pre}_jac_fd"),
                jacobian_fd_check(scene, res),
                "{:.1e}",
            )
            mc = monte_carlo(scene, res, 0.5, args.mc_samples)
            metrics[f"{robot}_mc"] = mc
            for k, v in mc.items():
                mac.add(_alpha(f"{pre}_mc_{k}"), v, "{:.3f}")
            # Height-error sensitivity of the fused estimate.
            shift = np.zeros(2)
            for c in res["hits"]:
                e = res["per"][c]
                dxy = e["d_world"][:2] / e["d_world"][2]
                shift += e["weight"] * dxy
            mac.add(
                _alpha(f"{pre}_dz_shift"),
                float(np.linalg.norm(shift)),
                "{:.3f}",
            )
            metrics[f"{robot}_example"] = ex

            ik = ik_residuals(scene, res["fused"])
            metrics[f"{robot}_ik"] = ik
            for k in ("hover_mm", "grasp_mm", "lift_mm"):
                mac.add(_alpha(f"{pre}_ik_{k}"), ik[k], "{:.3f}")
            fig_overview(scene, res["fused"])

            # Sweep table.
            rows = []
            for pose, sx, sy in spec["sweep"]:
                r = analyse_pose(scene, sx, sy)
                r["pose"] = pose
                r["xy"] = (sx, sy)
                rows.append(r)
                print(
                    f"{robot}/{pose}: hits={r['hits']} xy={r['xy_err_mm']:.3f}mm"
                )
                if robot == "omx" and pose == "mid_backish":
                    occ = fig_occlusion(
                        scene,
                        r["views"][0],
                        r["stages"][CAMS[0]],
                        r["dets"][CAMS[0]],
                        r["gt"],
                    )
                    metrics["omx_occlusion"] = occ
                    for k, v in occ.items():
                        mac.add(
                            _alpha(f"occ_{k}"),
                            v * 100 if k == "visible_frac" else v,
                            "{:.1f}" if k == "visible_frac" else "{:.2f}",
                        )
                    mono = r["per"][CAMS[1]]
                    mac.add(
                        "occMonoErr",
                        float(
                            np.linalg.norm(mono["point"][:2] - r["gt"][:2])
                            * 1000
                        ),
                        "{:.2f}",
                    )
                    left = r["per"][CAMS[0]]
                    mac.add(
                        "occLeftErr",
                        float(
                            np.linalg.norm(left["point"][:2] - r["gt"][:2])
                            * 1000
                        ),
                        "{:.2f}",
                    )
                    mac.add("occLeftW", left["weight"], "{:.3f}")
                    mac.add("occFusedErr", r["xy_err_mm"], "{:.2f}")
            tex_rows, md_rows = sweep_table(rows)
            mac.text[f"SWEEP_{robot.upper()}"] = md_rows
            (DOC_DIR / f"sweep_{robot}.tex").write_text(
                tex_rows + "\n", encoding="utf-8"
            )
            gaps = [
                abs(r["per"][c]["mec_gap"]) for r in rows for c in r["hits"]
            ]
            mac.add(_alpha(f"{pre}_mec_gap"), float(max(gaps)), "{:.1e}")
            finite = [
                r["xy_err_mm"] for r in rows if np.isfinite(r["xy_err_mm"])
            ]
            mac.add(_alpha(f"{pre}_sw_n"), len(rows), "{:.0f}")
            mac.add(_alpha(f"{pre}_sw_det"), len(finite), "{:.0f}")
            mac.add(
                _alpha(f"{pre}_sw_median"), float(np.median(finite)), "{:.2f}"
            )
            mac.add(_alpha(f"{pre}_sw_max"), float(np.max(finite)), "{:.2f}")
            metrics[f"{robot}_sweep"] = [
                {
                    "pose": r["pose"],
                    "xy": r["xy"],
                    "hits": r["hits"],
                    "xy_err_mm": r["xy_err_mm"],
                    "visible": {c: r["per"][c]["visible_frac"] for c in CAMS},
                }
                for r in rows
            ]
            fig_sensitivity(scene, [(sx, sy) for _, sx, sy in spec["sweep"]])

            if not args.skip_grid:
                gx = np.arange(*spec["grid_x"])
                gy = np.arange(
                    spec["grid_y"][0],
                    spec["grid_y"][1] + 1e-9,
                    spec["grid_y"][2],
                )
                grid_rows = []
                for x in gx:
                    for y in gy:
                        r = analyse_pose(
                            scene, float(x), float(y), with_seg=True
                        )
                        grid_rows.append(
                            {
                                "ball": r["gt"].tolist(),
                                "views": len(r["hits"]),
                                "xy_err_mm": r["xy_err_mm"],
                                "min_visible": min(
                                    r["per"][c]["visible_frac"] for c in CAMS
                                ),
                            }
                        )
                fig_error_map(scene, grid_rows)
                e = np.array([g["xy_err_mm"] for g in grid_rows])
                fin = e[np.isfinite(e)]
                full_vis = np.array(
                    [
                        g["xy_err_mm"]
                        for g in grid_rows
                        if np.isfinite(g["xy_err_mm"])
                        and g["min_visible"] > 0.95
                    ]
                )
                mac.add(_alpha(f"{pre}_grid_n"), e.size, "{:.0f}")
                mac.add(_alpha(f"{pre}_grid_det"), fin.size, "{:.0f}")
                mac.add(
                    _alpha(f"{pre}_grid_dual"),
                    sum(g["views"] == 2 for g in grid_rows),
                    "{:.0f}",
                )
                mac.add(
                    _alpha(f"{pre}_grid_median"),
                    float(np.median(fin)),
                    "{:.2f}",
                )
                mac.add(
                    _alpha(f"{pre}_grid_p95"),
                    float(np.percentile(fin, 95)),
                    "{:.2f}",
                )
                mac.add(
                    _alpha(f"{pre}_grid_max"), float(np.max(fin)), "{:.2f}"
                )
                mac.add(_alpha(f"{pre}_grid_clear_n"), full_vis.size, "{:.0f}")
                mac.add(
                    _alpha(f"{pre}_grid_clear_max"),
                    float(np.max(full_vis)) if full_vis.size else float("nan"),
                    "{:.2f}",
                )
                mac.add(
                    _alpha(f"{pre}_grid_clear_median"),
                    (
                        float(np.median(full_vis))
                        if full_vis.size
                        else float("nan")
                    ),
                    "{:.2f}",
                )
                metrics[f"{robot}_grid"] = grid_rows
    finally:
        for s in scenes.values():
            s.close()

    mac.write(DOC_DIR / "data.tex")
    render_markdown(mac)
    (DOC_DIR / "metrics.json").write_text(
        json.dumps(metrics, indent=1, default=float) + "\n", encoding="utf-8"
    )
    print(f"wrote {DOC_DIR / 'data.tex'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
