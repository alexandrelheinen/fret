"""Vision → pick-goal bridge for pick-and-place runners (v1.4 T14-03).

Captures gate cameras, runs ``fret.vision``, and refreshes **pick-side** joint
waypoints from :class:`~fret.vision.types.BallObservation`. Place / dispenser
joints stay on the scenario YAML (known fixture).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import numpy as np
import numpy.typing as npt

from fret.control.pick_place_fsm import PickPlaceWaypoints
from fret.vision.factory import build_hsv_plane_pipeline
from fret.vision.types import BallObservation

RobotId = Literal["omx", "omy"]

_DEFAULT_VISION_CFG: dict[RobotId, Path] = {
    "omx": Path("src/fret/config/vision/omx_portal_overhead.yml"),
    "omy": Path("src/fret/config/vision/omy_portal_overhead.yml"),
}

# OM-X link5 IK targets a foreshortened XY (gripper hangs below EE); matches
# the SC-v13b / gate-layout regen heuristic.
_OMX_EE_XY_SCALE = 0.72
_OMX_HOVER_Z_M = 0.20
_OMX_GRASP_Z_M = 0.175
_OMX_LIFT_Z_M = 0.22


def observe_ball_mujoco(
    model: Any,
    data: Any,
    *,
    robot: RobotId,
    vision_config: str | Path | None = None,
) -> BallObservation | None:
    """Render gate cameras and return a fused ``BallObservation`` (or None)."""
    from fret.simulation.mujoco_camera import MujocoCameraAdapter

    cfg_path = Path(vision_config or _DEFAULT_VISION_CFG[robot])
    pipeline = build_hsv_plane_pipeline(cfg_path)
    vision_cfg = pipeline.config
    if vision_cfg is None:
        raise RuntimeError(f"vision pipeline from {cfg_path} has no config")
    camera_ids = tuple(item.camera_id for item in vision_cfg.intrinsics)

    frames = []
    adapters = [
        MujocoCameraAdapter(model, data, camera_name=name)
        for name in camera_ids
    ]
    try:
        for adapter in adapters:
            frames.append(adapter.capture(timestamp=0.0).frame)
    finally:
        for adapter in adapters:
            adapter.close()
    return pipeline.process(frames)


def _omx_ik_xyz(
    xyz: npt.NDArray[np.float64],
    *,
    seeds: list[npt.NDArray[np.float64]],
) -> npt.NDArray[np.float64]:
    from fret.control.kinematics_open_manipulator_x import (
        OpenManipulatorXKinematics,
    )

    kin = OpenManipulatorXKinematics()
    best_q = seeds[0]
    best_err = 1e9
    target = np.asarray(xyz, dtype=np.float64).reshape(3)
    for seed in seeds:
        q = kin.inverse_kinematics(target, seed=seed)
        err = float(np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - target))
        if err < best_err:
            best_err = err
            best_q = np.asarray(q, dtype=np.float64)
    if best_err > 0.008:
        raise RuntimeError(
            f"OM-X pick IK poor at {target}: {best_err * 1000:.1f} mm"
        )
    return best_q


def refresh_pick_waypoints_omx(
    base: PickPlaceWaypoints,
    ball_world: npt.NDArray[np.float64],
    *,
    xy_scale: float = _OMX_EE_XY_SCALE,
    hover_z_m: float = _OMX_HOVER_Z_M,
    grasp_z_m: float = _OMX_GRASP_Z_M,
    lift_z_m: float = _OMX_LIFT_Z_M,
) -> PickPlaceWaypoints:
    """IK pick_hover / pick_grasp / lift_hover from ball XYZ; keep place side."""
    ball = np.asarray(ball_world, dtype=np.float64).reshape(3)
    xy = ball[:2] * float(xy_scale)
    seeds = [
        base.pick_grasp.copy(),
        base.pick_hover.copy(),
        base.idle.copy(),
        np.array([0.0, 0.5, -0.5, 0.5], dtype=np.float64),
    ]
    hover = _omx_ik_xyz(
        np.array([xy[0], xy[1], hover_z_m], dtype=np.float64), seeds=seeds
    )
    grasp = _omx_ik_xyz(
        np.array([xy[0], xy[1], grasp_z_m], dtype=np.float64),
        seeds=[hover, *seeds],
    )
    lift = _omx_ik_xyz(
        np.array([xy[0], xy[1], lift_z_m], dtype=np.float64),
        seeds=[hover, *seeds],
    )
    return base.with_pick(pick_hover=hover, pick_grasp=grasp, lift_hover=lift)


def refresh_pick_waypoints_omy(
    base: PickPlaceWaypoints,
    ball_world: npt.NDArray[np.float64],
    model: Any,
    data: Any,
    *,
    hover_clearance_m: float = 0.10,
    lift_clearance_m: float = 0.14,
) -> PickPlaceWaypoints:
    """Pad-mid IK for pick_hover / pick_grasp / lift_hover; keep place side."""
    import mujoco as mj

    from fret.control.omy_pad_mid_ik import (
        OMY_ARM_JOINTS,
        OMY_GRIPPER_PINCH,
        pad_mid_ik,
    )

    ball = np.asarray(ball_world, dtype=np.float64).reshape(3)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    grip_adr = int(
        model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "rh_r1")]
    )
    limits = np.array(
        [
            model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
            for n in OMY_ARM_JOINTS
        ],
        dtype=np.float64,
    )
    specs = [
        (
            "pick_hover",
            ball + np.array([0.0, 0.0, hover_clearance_m]),
            0.0,
        ),
        ("pick_grasp", ball.copy(), OMY_GRIPPER_PINCH),
        (
            "lift_hover",
            ball + np.array([0.0, 0.0, lift_clearance_m]),
            OMY_GRIPPER_PINCH,
        ),
    ]
    seeds = [
        base.pick_hover.copy(),
        base.pick_grasp.copy(),
        base.idle.copy(),
        np.array([-0.1001, 0.683, 2.0814, -0.1359, -0.7662, 0.0036]),
    ]
    out: dict[str, npt.NDArray[np.float64]] = {}
    seed = base.pick_hover.copy()
    for name, target, gv in specs:
        best_q: npt.NDArray[np.float64] | None = None
        best_err = 1e9
        for cand in [seed, *seeds]:
            q, err = pad_mid_ik(
                model,
                data,
                mj,
                target=np.asarray(target, dtype=np.float64),
                grip_val=float(gv),
                seed=np.asarray(cand, dtype=np.float64),
                limits=limits,
                pad_right=pad_right,
                pad_left=pad_left,
                grip_adr=grip_adr,
            )
            if err < best_err:
                best_q, best_err = q, err
        if best_q is None or best_err > 0.008:
            raise RuntimeError(
                f"OMY pad-mid pick IK poor at {name}: "
                f"{best_err * 1000:.1f} mm"
            )
        out[name] = best_q
        seed = best_q
    return base.with_pick(
        pick_hover=out["pick_hover"],
        pick_grasp=out["pick_grasp"],
        lift_hover=out["lift_hover"],
    )


def apply_vision_pick_goals(
    base: PickPlaceWaypoints,
    *,
    robot: RobotId,
    model: Any,
    data: Any,
    vision_config: str | Path | None = None,
) -> tuple[PickPlaceWaypoints, BallObservation]:
    """Observe the ball and return updated waypoints + the observation.

    Raises:
        RuntimeError: if the vision pipeline returns no ball.
    """
    observation = observe_ball_mujoco(
        model, data, robot=robot, vision_config=vision_config
    )
    if observation is None:
        raise RuntimeError(
            f"{robot} vision pipeline returned no BallObservation"
        )
    ball = np.asarray(observation.position_world, dtype=np.float64)
    if robot == "omx":
        refreshed = refresh_pick_waypoints_omx(base, ball)
    else:
        refreshed = refresh_pick_waypoints_omy(base, ball, model, data)
    return refreshed, observation
