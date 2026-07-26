"""Vision → pick-goal bridge for pick-and-place runners (v1.4 T14-03).

Captures gate cameras, runs ``fret.vision``, and refreshes **pick-side** joint
waypoints from :class:`~fret.vision.types.BallObservation`. Place / dispenser
joints stay on the scenario YAML (known fixture).
"""

from __future__ import annotations

from collections.abc import Sequence
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

# Floor grasp: pad midpoint slightly above ball centre so pads clear the
# table plane while still pinching the sphere.
_FLOOR_GRASP_PAD_LIFT_M = 0.012
_OMX_ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4")
_OMX_GRIPPER_OPEN = 0.019
_OMX_GRIPPER_PINCH = -0.01


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


def _pad_mid_pick_waypoints(
    base: PickPlaceWaypoints,
    ball_world: npt.NDArray[np.float64],
    model: Any,
    data: Any,
    *,
    arm_joints: tuple[str, ...],
    grip_joint: str,
    grip_open: float,
    grip_pinch: float,
    hover_clearance_m: float,
    lift_clearance_m: float,
    grasp_pad_lift_m: float = _FLOOR_GRASP_PAD_LIFT_M,
    extra_seeds: list[npt.NDArray[np.float64]] | None = None,
) -> PickPlaceWaypoints:
    """Shared pad-mid IK for floor (or pedestal) pick poses."""
    import mujoco as mj

    from fret.control.omy_pad_mid_ik import pad_mid_ik

    ball = np.asarray(ball_world, dtype=np.float64).reshape(3)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    grip_adr = int(
        model.jnt_qposadr[
            mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, grip_joint)
        ]
    )
    limits = np.array(
        [
            model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
            for n in arm_joints
        ],
        dtype=np.float64,
    )
    grasp_target = ball + np.array([0.0, 0.0, grasp_pad_lift_m])
    specs = [
        (
            "pick_hover",
            ball + np.array([0.0, 0.0, hover_clearance_m]),
            grip_open,
        ),
        ("pick_grasp", grasp_target, grip_pinch),
        (
            "lift_hover",
            ball + np.array([0.0, 0.0, lift_clearance_m]),
            grip_pinch,
        ),
    ]
    seeds = [
        base.pick_hover.copy(),
        base.pick_grasp.copy(),
        base.idle.copy(),
    ]
    if extra_seeds:
        seeds.extend(extra_seeds)
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
                arm_joints=arm_joints,
            )
            if err < best_err:
                best_q, best_err = q, err
        if best_q is None or best_err > 0.008:
            raise RuntimeError(
                f"pad-mid pick IK poor at {name}: {best_err * 1000:.1f} mm"
            )
        out[name] = best_q
        seed = best_q
    return base.with_pick(
        pick_hover=out["pick_hover"],
        pick_grasp=out["pick_grasp"],
        lift_hover=out["lift_hover"],
    )


def refresh_pick_waypoints_omx(
    base: PickPlaceWaypoints,
    ball_world: npt.NDArray[np.float64],
    model: Any,
    data: Any,
    *,
    hover_clearance_m: float = 0.08,
    lift_clearance_m: float = 0.12,
    grasp_pad_lift_m: float = _FLOOR_GRASP_PAD_LIFT_M,
) -> PickPlaceWaypoints:
    """Pad-mid IK for OM-X pick poses (floor-safe); keep place side."""
    return _pad_mid_pick_waypoints(
        base,
        ball_world,
        model,
        data,
        arm_joints=_OMX_ARM_JOINTS,
        grip_joint="Gripper",
        grip_open=_OMX_GRIPPER_OPEN,
        grip_pinch=_OMX_GRIPPER_PINCH,
        hover_clearance_m=hover_clearance_m,
        lift_clearance_m=lift_clearance_m,
        grasp_pad_lift_m=grasp_pad_lift_m,
        extra_seeds=[
            np.array([-0.78, 0.50, -0.50, 0.65], dtype=np.float64),
            np.array([-0.78, 0.70, -0.20, 0.65], dtype=np.float64),
        ],
    )


def refresh_pick_waypoints_omy(
    base: PickPlaceWaypoints,
    ball_world: npt.NDArray[np.float64],
    model: Any,
    data: Any,
    *,
    hover_clearance_m: float = 0.10,
    lift_clearance_m: float = 0.14,
    grasp_pad_lift_m: float = _FLOOR_GRASP_PAD_LIFT_M,
) -> PickPlaceWaypoints:
    """Pad-mid IK for OMY pick poses (floor-safe); keep place side."""
    from fret.control.omy_pad_mid_ik import OMY_ARM_JOINTS, OMY_GRIPPER_PINCH

    return _pad_mid_pick_waypoints(
        base,
        ball_world,
        model,
        data,
        arm_joints=OMY_ARM_JOINTS,
        grip_joint="rh_r1",
        grip_open=0.0,
        grip_pinch=OMY_GRIPPER_PINCH,
        hover_clearance_m=hover_clearance_m,
        lift_clearance_m=lift_clearance_m,
        grasp_pad_lift_m=grasp_pad_lift_m,
        extra_seeds=[
            np.array([-0.1001, 0.683, 2.0814, -0.1359, -0.7662, 0.0036]),
            np.array([-0.15, 0.75, 2.05, -0.25, -0.70, 0.0]),
        ],
    )


def set_cv_ball_ghost(
    model: Any,
    data: Any,
    position_world: npt.NDArray[np.float64] | Sequence[float],
    *,
    rgba: tuple[float, float, float, float] = (0.90, 0.10, 0.10, 0.35),
) -> bool:
    """Move the non-colliding red CV ghost sphere to ``position_world``.

    The geom is visual-only (``contype=0``). Missing body/geom is a no-op so
    older MJCF templates without the ghost still load.
    """
    import mujoco as mj

    body_id = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "cv_ball_ghost"))
    if body_id < 0:
        return False
    pos = np.asarray(position_world, dtype=np.float64).reshape(3)
    model.body_pos[body_id] = pos
    geom_id = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "cv_ball_ghost"))
    if geom_id >= 0:
        model.geom_rgba[geom_id] = np.asarray(rgba, dtype=np.float64)
    mj.mj_forward(model, data)
    return True


def apply_vision_pick_goals(
    base: PickPlaceWaypoints,
    *,
    robot: RobotId,
    model: Any,
    data: Any,
    vision_config: str | Path | None = None,
) -> tuple[PickPlaceWaypoints, BallObservation]:
    """Observe the ball and return updated waypoints + the observation.

    Also places the transparent red ``cv_ball_ghost`` at the CV estimate for
    overview proof renders.

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
    set_cv_ball_ghost(model, data, ball)
    if robot == "omx":
        refreshed = refresh_pick_waypoints_omx(base, ball, model, data)
    else:
        refreshed = refresh_pick_waypoints_omy(base, ball, model, data)
    return refreshed, observation
