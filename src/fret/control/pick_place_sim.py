"""MuJoCo runner for SC-v13b pick-and-place (FSM + free box).

Finger-mesh pinch with stock Menagerie colliders is unreliable, so while the
FSM reports a held state the box freejoint is kinematically attached to
``link5``. The box remains a free body before grasp and after release.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import numpy.typing as npt

from fret.control.pick_place_fsm import (
    GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
    PickPlaceWaypoints,
)
from fret.sitl_config import load_scenario_parameters, mjcf_path

_HOLD_STATES = frozenset(
    {
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
        PickPlaceState.RELEASE,  # keep attached until RELEASE hold finishes
    }
)


def waypoints_from_scenario(
    scenario_path: str | Path | None = None,
) -> PickPlaceWaypoints:
    """Load joint waypoints from ``omx_pick_place.yml``."""
    path = Path(
        scenario_path or "src/fret/config/scenarios/omx_pick_place.yml"
    )
    p = load_scenario_parameters(path)
    return PickPlaceWaypoints(
        idle=np.asarray(p["idle_configuration"], dtype=np.float64),
        pick_hover=np.asarray(p["pick_hover_configuration"], dtype=np.float64),
        pick_grasp=np.asarray(p["pick_grasp_configuration"], dtype=np.float64),
        place_hover=np.asarray(
            p["place_hover_configuration"], dtype=np.float64
        ),
        place_grasp=np.asarray(
            p["place_grasp_configuration"], dtype=np.float64
        ),
    )


def _arm_q(
    mujoco: object, model: object, data: object
) -> npt.NDArray[np.float64]:
    return np.array(
        [
            data.qpos[
                model.jnt_qposadr[
                    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                ]
            ]
            for name in ("Joint1", "Joint2", "Joint3", "Joint4")
        ],
        dtype=np.float64,
    )


def _attach_box(
    mujoco: object,
    model: object,
    data: object,
    *,
    ee_body: int,
    box_qadr: int,
    box_dadr: int,
    offset_local: npt.NDArray[np.float64],
) -> None:
    """Write the box freejoint pose so it follows the EE with a fixed offset."""
    mujoco.mj_forward(model, data)
    rot = data.xmat[ee_body].reshape(3, 3)
    pos = data.xpos[ee_body] + rot @ offset_local
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, data.xmat[ee_body])
    data.qpos[box_qadr : box_qadr + 3] = pos
    data.qpos[box_qadr + 3 : box_qadr + 7] = quat
    data.qvel[box_dadr : box_dadr + 6] = 0.0


def run_pick_place(
    *,
    duration_s: float = 20.0,
    joint_tol_rad: float = 0.10,
) -> tuple[PickPlaceState, npt.NDArray[np.float64]]:
    """Execute one SC-v13b cycle; return final FSM state and box position."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for pick-place sim") from exc

    wp = waypoints_from_scenario()
    xml = mjcf_path("open_manipulator_x", "omx_pick_place")
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)

    ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link5")
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])
    box_dadr = int(model.jnt_dofadr[box_jid])

    fsm = PickPlaceFSM(
        wp,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=0.5,
        release_hold_s=0.4,
        lift_height_m=0.12,
        phase_timeout_s=15.0,
    )
    fsm.start()

    data.ctrl[:4] = wp.idle
    data.ctrl[4] = GRIPPER_OPEN
    for _ in range(400):
        mj.mj_step(model, data)

    offset: npt.NDArray[np.float64] | None = None
    holding = False
    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)

    for _ in range(max_steps):
        obs = PickPlaceObservation(
            q=_arm_q(mj, model, data),
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=np.asarray(data.xpos[ee_id], dtype=np.float64).copy(),
        )
        prev = fsm.state
        cmd = fsm.tick(obs, dt)

        # Capture grasp offset when entering LIFT; attach through transfer.
        if (
            prev == PickPlaceState.GRASP
            and cmd.state == PickPlaceState.LIFT
            and offset is None
        ):
            mj.mj_forward(model, data)
            rot = data.xmat[ee_id].reshape(3, 3)
            offset = rot.T @ (data.xpos[box_id] - data.xpos[ee_id])
            holding = True

        if cmd.state == PickPlaceState.RETREAT:
            holding = False
            offset = None

        data.ctrl[:4] = cmd.q_des
        data.ctrl[4] = cmd.gripper
        if holding and offset is not None and cmd.state in _HOLD_STATES:
            _attach_box(
                mj,
                model,
                data,
                ee_body=ee_id,
                box_qadr=box_qadr,
                box_dadr=box_dadr,
                offset_local=offset,
            )
        mj.mj_step(model, data)

        if fsm.state in {PickPlaceState.DONE, PickPlaceState.FAULT}:
            break

    return fsm.state, np.asarray(data.xpos[box_id], dtype=np.float64).copy()
