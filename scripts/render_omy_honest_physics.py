#!/usr/bin/env python3
"""Render an OMY pick-place MP4 from recorded physics states (no kinematic cheats).

Phase 1 — run ``simulate_omy_pick_place`` (MuJoCo integrator only; no attach).
Phase 2 — render each recorded sample for the MP4 (visualization replay).

Replay sets ``qpos`` from the recorded trace; it does **not** change how the
trace was produced. Any ball teleport would show up as
``max_ball_step_jump_m`` in the metadata.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO / "scripts") not in sys.path:
    sys.path.insert(0, str(_REPO / "scripts"))

from render_mujoco import (  # noqa: E402
    _frame_mean,
    _open_video_writer,
    _require_mujoco,
    list_showcase_cameras,
)


def _max_ball_step_jump_m(samples: list[Any]) -> float:
    max_jump = 0.0
    for prev, cur in zip(samples, samples[1:]):
        jump = float(np.linalg.norm(cur.box_qpos[:3] - prev.box_qpos[:3]))
        max_jump = max(max_jump, jump)
    return max_jump


def render_honest_omy_pick_place(
    output_path: Path,
    *,
    scenario: str = "omy_pick_place",
    camera: str = "overview",
    duration_s: float = 130.0,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
) -> dict[str, object]:
    """Simulate physics, then render the recorded state trace to MP4."""
    mujoco, _iio = _require_mujoco()

    from fret.control.omy_pick_place_sim import simulate_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState
    from fret.sitl_config import load_scenario_parameters, mjcf_path

    scenario_yaml = Path("src/fret/config/scenarios") / f"{scenario}.yml"
    params = load_scenario_parameters(scenario_yaml)
    xml = mjcf_path("omy", scenario)
    cameras = list_showcase_cameras(xml, scenario=scenario)
    if camera not in cameras:
        raise ValueError(f"camera {camera!r} not in {cameras}")

    model_probe = mujoco.MjModel.from_xml_path(str(xml))
    dt = float(model_probe.opt.timestep)
    record_every = max(1, int(round((1.0 / fps) / dt)))

    state, samples = simulate_omy_pick_place(
        duration_s=duration_s,
        joint_tol_rad=0.16,
        record_every_steps=record_every,
        scenario_path=scenario_yaml,
    )
    if len(samples) < 2:
        raise RuntimeError("honest physics render: too few samples")

    states_seen: list[str] = []
    prev: PickPlaceState | None = None
    for sample in samples:
        if sample.state != prev:
            states_seen.append(sample.state.name)
            prev = sample.state

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])
    arm_names = (
        "Joint1",
        "Joint2",
        "Joint3",
        "Joint4",
        "Joint5",
        "Joint6",
    )

    renderer = mujoco.Renderer(model, height=height, width=width)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = _open_video_writer(output_path, fps)
    first_mean = 0.0
    try:
        for i, sample in enumerate(samples):
            for name, value in zip(arm_names, sample.q_arm, strict=True):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                data.qpos[model.jnt_qposadr[jid]] = float(value)
            for gname in ("rh_r1", "rh_l1", "rh_l2"):
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, gname)
                if gid >= 0:
                    data.qpos[model.jnt_qposadr[gid]] = float(sample.gripper)
            data.qpos[box_qadr : box_qadr + 7] = np.asarray(
                sample.box_qpos, dtype=np.float64
            )
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            writer.append_data(frame)
            if i == 0:
                first_mean = _frame_mean(frame)
    finally:
        writer.close()
        renderer.close()

    if first_mean <= 1.0:
        raise RuntimeError(f"render looks blank (frame mean={first_mean:.2f})")

    ball_z = np.array([s.box_qpos[2] for s in samples], dtype=np.float64)
    meta: dict[str, Any] = {
        "scenario": scenario,
        "camera": camera,
        "duration_s_requested": duration_s,
        "video_duration_s": float(len(samples)) / float(fps),
        "fps": fps,
        "frames": len(samples),
        "final_fsm_state": state.name,
        "fsm_state_sequence": states_seen,
        "ball_z_min_m": float(ball_z.min()),
        "ball_z_max_m": float(ball_z.max()),
        "max_ball_step_jump_m": _max_ball_step_jump_m(samples),
        "lift_height_target_m": float(params.get("lift_height_m", 0.2)),
        "physics_mode": "simulate_then_replay_recorded_states",
        "kinematic_attach": False,
        "output_mp4": str(output_path),
    }
    return meta


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Honest OMY pick-place physics MP4 from recorded states."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("/opt/cursor/artifacts/omy_pick_place_honest_physics.mp4"),
    )
    parser.add_argument("--scenario", default="omy_pick_place")
    parser.add_argument("--camera", default="overview")
    parser.add_argument("--duration-s", type=float, default=130.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--meta-json", type=Path, default=None)
    args = parser.parse_args()

    meta = render_honest_omy_pick_place(
        args.output,
        scenario=args.scenario,
        camera=args.camera,
        duration_s=args.duration_s,
        fps=args.fps,
        width=args.width,
        height=args.height,
    )
    meta_path = args.meta_json or args.output.with_suffix(".json")
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
