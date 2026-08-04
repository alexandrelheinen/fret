#!/usr/bin/env python3
"""Render a short MP4 proving place-bin walls block the proximal arm.

Clip A: drive the historical elbow-down dunk into the bin — link4 contacts
``place_bin_wall_nx`` (penetration depth logged on stderr).

Clip B: replay the place-phase samples from a successful OMY pick-place run —
arm stays outside the colliding walls while the ball settles in the funnel.
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


def _set_arm(
    mujoco: object,
    model: object,
    data: object,
    q: np.ndarray,
    *,
    grip: float,
) -> None:
    joints = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
    for name, val in zip(joints, q, strict=True):
        adr = int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        data.qpos[adr] = float(val)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid >= 0:
            data.ctrl[aid] = float(val)
    grip_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Gripper")
    if grip_act >= 0:
        data.ctrl[grip_act] = float(grip)
    mujoco.mj_forward(model, data)


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


def _clip_dunk_blocked(
    mujoco: object,
    model: object,
    data: object,
    renderer: object,
    writer: object,
    *,
    fps: int,
    seconds: float,
) -> tuple[int, float]:
    """Interpolate toward the old elbow-down dunk; physics must hit the wall."""
    start = np.array(
        [0.6553, 0.0952, 1.2659, -0.5150, 0.5410, 0.1035], dtype=np.float64
    )
    dunk = np.array(
        [0.4298, 0.3654, 1.8952, -0.2575, -0.9997, -0.0128], dtype=np.float64
    )
    cam = _place_camera(mujoco, model, data)
    frames = max(2, int(round(seconds * fps)))
    max_hits = 0
    worst = 0.0
    for i in range(frames):
        alpha = min(1.0, float(i) / max(1, frames - 1))
        q = (1.0 - alpha) * start + alpha * dunk
        _set_arm(mujoco, model, data, q, grip=1.05)
        for _ in range(4):
            mujoco.mj_step(model, data)
        hits, depth = _bin_contact_summary(mujoco, model, data)
        max_hits = max(max_hits, hits)
        worst = min(worst, depth)
        renderer.update_scene(data, camera=cam)
        writer.append_data(renderer.render())
    return max_hits, worst


def _clip_successful_place(
    mujoco: object,
    model: object,
    data: object,
    renderer: object,
    writer: object,
    *,
    fps: int,
) -> None:
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
        raise RuntimeError(f"place replay ended in {state.name}, expected DONE")

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
    cam = _place_camera(mujoco, model, data)

    # Keep transfer + place phases only (last ~40% of motion).
    place_samples = [
        s
        for s in samples
        if s.state.name
        in {
            "MOVE_PLACE",
            "DESCEND_PLACE",
            "RELEASE",
            "RETREAT",
            "DONE",
        }
    ]
    if len(place_samples) < 30:
        place_samples = samples[-max(30, len(samples) // 3) :]

    stride = max(1, len(place_samples) // int(8.0 * fps))
    picked = place_samples[::stride]
    if picked[-1] is not place_samples[-1]:
        picked.append(place_samples[-1])

    for sample in picked:
        for name in joints:
            data.qpos[qadr[name]] = float(sample.q_arm[joints.index(name)])
            aid = act[name]
            if aid >= 0:
                data.ctrl[aid] = float(sample.q_arm[joints.index(name)])
        if grip_act >= 0:
            data.ctrl[grip_act] = float(sample.gripper)
        data.qpos[box_qadr : box_qadr + 7] = sample.box_qpos
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        writer.append_data(renderer.render())


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
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--dunk-seconds",
        type=float,
        default=3.0,
        help="Length of the forced dunk segment",
    )
    args = parser.parse_args()

    mujoco, _mj = _require_mujoco()
    from fret.mjcf.omy import ensure_omy_pick_place_mjcf

    xml = ensure_omy_pick_place_mjcf()
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    writer = _open_writer(args.output, args.fps)
    try:
        hits, depth = _clip_dunk_blocked(
            mujoco,
            model,
            data,
            renderer,
            writer,
            fps=args.fps,
            seconds=args.dunk_seconds,
        )
        _clip_successful_place(
            mujoco, model, data, renderer, writer, fps=args.fps
        )
    finally:
        writer.close()
        renderer.close()

    print(
        f"Wrote {args.output} (dunk place_bin contacts={hits}, "
        f"min_depth={depth:.4f} m)",
        flush=True,
    )


if __name__ == "__main__":
    main()
