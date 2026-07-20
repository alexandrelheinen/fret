#!/usr/bin/env python3
"""Compute OMY pad-mid IK waypoints for ground-ball pick-place scenarios.

Targets the midpoint between injected finger pads (not link6). Uses sequential
numerical IK on the Menagerie model with pads injected via
:func:`fret.mjcf.omy.ensure_omy_pick_place_mjcf`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from fret.mjcf.omy import ensure_omy_pick_place_mjcf

_ARM = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
_GRIPPER_OPEN = 0.0
_GRIPPER_PINCH = 1.05


def _pad_mid_ik(
    model: object,
    data: object,
    mj: object,
    *,
    target: np.ndarray,
    grip_val: float,
    seed: np.ndarray,
    limits: np.ndarray,
    pad_right: int,
    pad_left: int,
    grip_adr: int,
) -> tuple[np.ndarray, float]:
    """Return joint vector placing pad-mid at ``target`` (kinematic FK)."""
    target = np.asarray(target, dtype=np.float64)

    def set_kin(q: np.ndarray) -> None:
        for i, name in enumerate(_ARM):
            adr = int(
                model.jnt_qposadr[  # type: ignore[attr-defined]
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                ]
            )
            data.qpos[adr] = float(q[i])  # type: ignore[attr-defined]
        data.qpos[grip_adr] = float(grip_val)  # type: ignore[attr-defined]
        mj.mj_forward(model, data)

    def midpoint() -> np.ndarray:
        return 0.5 * (
            np.asarray(data.geom_xpos[pad_right], dtype=np.float64)  # type: ignore[attr-defined]
            + np.asarray(data.geom_xpos[pad_left], dtype=np.float64)  # type: ignore[attr-defined]
        )

    def cost(q: np.ndarray) -> float:
        set_kin(q)
        return float(np.sum((midpoint() - target) ** 2))

    res = minimize(
        cost,
        np.asarray(seed, dtype=np.float64),
        method="L-BFGS-B",
        bounds=limits,
        options={"maxiter": 500, "ftol": 1e-14},
    )
    set_kin(res.x)
    err = float(np.linalg.norm(midpoint() - target))
    return res.x.astype(np.float64), err


def _physics_refine(
    model: object,
    data: object,
    mj: object,
    *,
    seed: np.ndarray,
    grip_val: float,
    ball_pos: np.ndarray,
    limits: np.ndarray,
    pad_right: int,
    pad_left: int,
    arm_act: list[int],
    grip_act: int,
    box_body: int,
    box_qadr: int,
) -> tuple[np.ndarray, float]:
    """Refine ``seed`` so physics pad-mid seats on the ball centre."""

    def cost(q: np.ndarray) -> float:
        data.qpos[box_qadr : box_qadr + 3] = ball_pos  # type: ignore[attr-defined]
        data.qpos[box_qadr + 3 : box_qadr + 7] = np.array(  # type: ignore[attr-defined]
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )
        data.qvel[box_qadr : box_qadr + 6] = 0.0  # type: ignore[attr-defined]
        for i, aid in enumerate(arm_act):
            data.ctrl[aid] = float(q[i])  # type: ignore[attr-defined]
        data.ctrl[grip_act] = float(grip_val)  # type: ignore[attr-defined]
        for _ in range(900):
            mj.mj_step(model, data)
        mid = 0.5 * (
            np.asarray(data.geom_xpos[pad_right], dtype=np.float64)  # type: ignore[attr-defined]
            + np.asarray(data.geom_xpos[pad_left], dtype=np.float64)  # type: ignore[attr-defined]
        )
        ball = np.asarray(data.xpos[box_body], dtype=np.float64)  # type: ignore[attr-defined]
        return float(np.sum((mid - ball) ** 2))

    res = minimize(
        cost,
        np.asarray(seed, dtype=np.float64),
        method="L-BFGS-B",
        bounds=limits,
        options={"maxiter": 40, "ftol": 1e-10},
    )
    err = float(np.sqrt(cost(res.x)))
    return res.x.astype(np.float64), err


def compute_waypoints(
    *,
    pick_xy: tuple[float, float] = (0.40, -0.28),
    place_xy: tuple[float, float] = (0.40, 0.28),
    ball_radius_m: float = 0.043,
    gripper_pinch: float = _GRIPPER_PINCH,
    idle: np.ndarray | None = None,
) -> dict[str, list[float]]:
    """Return named joint waypoints for one ground-ball pick-place cycle."""
    import mujoco as mj

    xml = ensure_omy_pick_place_mjcf()
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    box_body = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box"))
    box_j = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint"))
    box_qadr = int(model.jnt_qposadr[box_j])
    grip_j = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "rh_r1"))
    grip_adr = int(model.jnt_qposadr[grip_j])
    grip_act = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "Gripper"))
    arm_act = [
        int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name))
        for name in _ARM
    ]
    limits = np.array(
        [
            model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)]
            for name in _ARM
        ],
        dtype=np.float64,
    )

    ball_pick = np.array(
        [pick_xy[0], pick_xy[1], ball_radius_m], dtype=np.float64
    )
    ball_place = np.array(
        [place_xy[0], place_xy[1], ball_radius_m], dtype=np.float64
    )
    # Folded home (distinct from hover) — mirrors OMX idle vs approach.
    seed = (
        np.asarray(idle, dtype=np.float64)
        if idle is not None
        else np.array([0.0, -0.9, 1.4, 0.0, 0.6, 0.0], dtype=np.float64)
    )

    idle_target = np.array([0.20, 0.0, 0.35], dtype=np.float64)
    specs: list[tuple[str, np.ndarray, float]] = [
        ("idle", idle_target, _GRIPPER_OPEN),
        ("pick_hover", ball_pick + np.array([0.0, 0.0, 0.10]), _GRIPPER_OPEN),
        ("pick_grasp", ball_pick.copy(), gripper_pinch),
        ("lift_hover", ball_pick + np.array([0.0, 0.0, 0.14]), gripper_pinch),
        (
            "place_hover",
            ball_place + np.array([0.0, 0.0, 0.14]),
            gripper_pinch,
        ),
        (
            "place_grasp",
            ball_place + np.array([0.0, 0.0, 0.06]),
            gripper_pinch,
        ),
    ]
    out: dict[str, list[float]] = {}
    for name, target, gv in specs:
        q, err = _pad_mid_ik(
            model,
            data,
            mj,
            target=target,
            grip_val=gv,
            seed=seed,
            limits=limits,
            pad_right=pad_right,
            pad_left=pad_left,
            grip_adr=grip_adr,
        )
        if err > 0.008:
            raise RuntimeError(f"{name} pad-mid IK failed (err={err:.4f} m)")
        ball_pos = (
            ball_pick
            if name.startswith("pick") or name in {"lift_hover", "idle"}
            else ball_place
        )
        if name != "idle":
            q, perr = _physics_refine(
                model,
                data,
                mj,
                seed=q,
                grip_val=gv,
                ball_pos=ball_pos,
                limits=limits,
                pad_right=pad_right,
                pad_left=pad_left,
                arm_act=arm_act,
                grip_act=grip_act,
                box_body=box_body,
                box_qadr=box_qadr,
            )
            if name == "pick_grasp" and perr > 0.025:
                raise RuntimeError(
                    f"{name} physics pad-seat failed (err={perr * 1000:.1f} mm)"
                )
        out[name] = [round(float(v), 4) for v in q]
        seed = q
    out["retreat"] = list(out["idle"])
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ball-radius-m",
        type=float,
        default=0.043,
        help="Floor-ball radius in metres (SC-v14b default 0.043)",
    )
    args = parser.parse_args()
    wp = compute_waypoints(ball_radius_m=float(args.ball_radius_m))
    for key, val in wp.items():
        print(f"{key}: {val}")


if __name__ == "__main__":
    main()
