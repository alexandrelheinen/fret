#!/usr/bin/env python3
"""Render an MP4 of the normal OMY pick-place flow at real-time speed.

Replays a successful ``simulate_omy_pick_place`` run with physics stepping
(actuators + MuJoCo integration) so the clip shows the full pick → transfer →
place cycle without forced dunk poses or time compression.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO / "src") not in sys.path:
    sys.path.insert(0, str(_REPO / "src"))


def _require_mujoco() -> tuple[object, object]:
    import os

    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    import mujoco

    return mujoco, mujoco


def _open_writer(path: Path, fps: int) -> object:
    import imageio

    path.parent.mkdir(parents=True, exist_ok=True)
    return imageio.get_writer(
        str(path),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
        macro_block_size=1,
    )


def _place_camera(mujoco: object, model: object, data: object) -> object:
    """Close isometric view of the place bin (+X side)."""
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = np.array([0.48, 0.0, 0.18], dtype=np.float64)
    cam.distance = 0.95
    cam.azimuth = 118.0
    cam.elevation = -18.0
    return cam


def _overview_camera(mujoco: object) -> object:
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = np.array([0.20, -0.05, 0.20], dtype=np.float64)
    cam.distance = 1.55
    cam.azimuth = 132.0
    cam.elevation = -22.0
    return cam


def _seed_sample(
    mujoco: object,
    model: object,
    data: object,
    sample: object,
    *,
    joints: tuple[str, ...],
    qadr: dict[str, int],
    box_qadr: int,
) -> None:
    for name in joints:
        data.qpos[qadr[name]] = float(sample.q_arm[joints.index(name)])
    grip_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rh_r1")
    if grip_j >= 0:
        data.qpos[int(model.jnt_qposadr[grip_j])] = float(sample.gripper)
    data.qpos[box_qadr : box_qadr + 7] = sample.box_qpos
    mujoco.mj_forward(model, data)


def _physics_step_sample(
    mujoco: object,
    model: object,
    data: object,
    sample: object,
    *,
    joints: tuple[str, ...],
    act: dict[str, int],
    grip_act: int,
) -> None:
    for name in joints:
        aid = act[name]
        if aid >= 0:
            data.ctrl[aid] = float(sample.q_arm[joints.index(name)])
    if grip_act >= 0:
        data.ctrl[grip_act] = float(sample.gripper)
    mujoco.mj_step(model, data)


def _bin_contact_summary(
    mujoco: object, model: object, data: object
) -> tuple[int, float]:
    names = [model.geom(i).name for i in range(model.ngeom)]
    worst = 0.0
    hits = 0
    for k in range(int(data.ncon)):
        c = data.contact[k]
        g1, g2 = names[c.geom1], names[c.geom2]
        if (g1 and g1.startswith("place_bin")) or (
            g2 and g2.startswith("place_bin")
        ):
            hits += 1
            worst = min(worst, float(c.dist))
    return hits, worst


def _render_normal_flow(
    mujoco: object,
    model: object,
    data: object,
    renderer: object,
    writer: object,
    *,
    camera: object,
    fps: int,
) -> tuple[int, float]:
    from fret.control.omy_pick_place_sim import simulate_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    scenario = Path("src/fret/config/scenarios/omy_pick_place.yml")
    state, samples = simulate_omy_pick_place(
        duration_s=55.0,
        joint_tol_rad=0.22,
        record_every_steps=1,
        scenario_path=scenario,
        use_vision=False,
    )
    if state != PickPlaceState.DONE:
        raise RuntimeError(f"pick-place replay ended in {state.name}, expected DONE")
    if len(samples) < 2:
        raise RuntimeError("pick-place replay recorded too few frames")

    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    box_qadr = int(model.jnt_qposadr[box_jid])
    joints = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
    qadr = {
        name: int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        for name in joints
    }
    act = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in joints
    }
    grip_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Gripper")

    _seed_sample(
        mujoco,
        model,
        data,
        samples[0],
        joints=joints,
        qadr=qadr,
        box_qadr=box_qadr,
    )
    renderer.update_scene(data, camera=camera)
    writer.append_data(renderer.render())

    max_hits = 0
    worst = 0.0
    for sample in samples[1:]:
        _physics_step_sample(
            mujoco,
            model,
            data,
            sample,
            joints=joints,
            act=act,
            grip_act=grip_act,
        )
        hits, depth = _bin_contact_summary(mujoco, model, data)
        max_hits = max(max_hits, hits)
        worst = min(worst, depth)
        renderer.update_scene(data, camera=camera)
        writer.append_data(renderer.render())

    return max_hits, worst


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(
            "/opt/cursor/artifacts/place_bin_collision_proof/"
            "omy_place_bin_collision_proof.mp4"
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=0,
        help="Output FPS (0 = match MuJoCo timestep, real-time)",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--camera",
        choices=("place", "overview"),
        default="place",
        help="Camera framing (default: place-bin isometric)",
    )
    args = parser.parse_args()

    mujoco, _mj = _require_mujoco()
    from fret.mjcf.omy import ensure_omy_pick_place_mjcf

    xml = ensure_omy_pick_place_mjcf()
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    fps = int(args.fps) if int(args.fps) > 0 else max(1, int(round(1.0 / dt)))

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    writer = _open_writer(args.output, fps)
    camera = (
        _place_camera(mujoco, model, data)
        if args.camera == "place"
        else _overview_camera(mujoco)
    )
    try:
        hits, depth = _render_normal_flow(
            mujoco,
            model,
            data,
            renderer,
            writer,
            camera=camera,
            fps=fps,
        )
    finally:
        writer.close()
        renderer.close()

    print(
        f"Wrote {args.output} ({fps} fps, place_bin contacts={hits}, "
        f"min_depth={depth:.4f} m)",
        flush=True,
    )


if __name__ == "__main__":
    main()
