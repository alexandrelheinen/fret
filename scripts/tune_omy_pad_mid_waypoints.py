#!/usr/bin/env python3
"""Compute OMY pad-mid IK waypoints for pedestal-ball pick-place scenarios.

Targets the midpoint between injected finger pads (not link6). Uses sequential
numerical IK on the Menagerie model with pads injected via
:func:`fret.mjcf.omy.ensure_omy_pick_place_mjcf`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from fret.control.omy_pad_mid_ik import (
    OMY_ARM_JOINTS as _ARM,
    OMY_GRIPPER_PINCH as _GRIPPER_PINCH,
    pad_mid_ik as _pad_mid_ik,
)
from fret.mjcf.omy import ensure_omy_pick_place_mjcf

_GRIPPER_OPEN = 0.0


def _physics_pad_contact(
    model: object,
    data: object,
    mj: object,
    *,
    seed: np.ndarray,
    grip_val: float,
    arm_act: list[int],
    grip_act: int,
    box_body: int,
) -> tuple[bool, float]:
    """Settle under ctrl and report pad↔ball contact + pad-mid distance."""
    for i, aid in enumerate(arm_act):
        data.ctrl[aid] = float(seed[i])  # type: ignore[attr-defined]
    data.ctrl[grip_act] = float(grip_val)  # type: ignore[attr-defined]
    for _ in range(1200):
        mj.mj_step(model, data)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    mid = 0.5 * (
        np.asarray(data.geom_xpos[pad_right], dtype=np.float64)  # type: ignore[attr-defined]
        + np.asarray(data.geom_xpos[pad_left], dtype=np.float64)  # type: ignore[attr-defined]
    )
    ball = np.asarray(data.xpos[box_body], dtype=np.float64)  # type: ignore[attr-defined]
    dist = float(np.linalg.norm(mid - ball))
    box_geom_adr = int(model.body_geomadr[box_body])  # type: ignore[attr-defined]
    box_geom_num = int(model.body_geomnum[box_body])  # type: ignore[attr-defined]
    box_geoms = frozenset(range(box_geom_adr, box_geom_adr + box_geom_num))
    contact = False
    for ci in range(int(data.ncon)):  # type: ignore[attr-defined]
        c = data.contact[ci]  # type: ignore[attr-defined]
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 not in box_geoms and g2 not in box_geoms:
            continue
        other = g2 if g1 in box_geoms else g1
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, other) or ""
        if name.startswith("pad_"):
            contact = True
            break
    return contact, dist


def compute_waypoints(
    *,
    pick_xy: tuple[float, float] = (0.40, -0.28),
    place_xy: tuple[float, float] = (0.50, 0.20),
    ball_radius_m: float = 0.043,
    cone_height_m: float = 0.280,
    # Half end-effector package (~pads+fingers) above the rim for the drop.
    # Target slightly high so residual IK error still clears the rim.
    place_drop_clearance_m: float = 0.050,
    ball_pick_z_m: float | None = None,
    gripper_pinch: float = _GRIPPER_PINCH,
    idle: np.ndarray | None = None,
) -> dict[str, list[float]]:
    """Return named joint waypoints for one pedestal-ball pick-place cycle."""
    import mujoco as mj

    xml = ensure_omy_pick_place_mjcf()
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    box_body = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box"))
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

    mj.mj_forward(model, data)
    pick_z = (
        float(ball_pick_z_m)
        if ball_pick_z_m is not None
        else float(data.xpos[box_body][2])
    )
    ball_pick = np.array([pick_xy[0], pick_xy[1], pick_z], dtype=np.float64)
    # Drop from above the rim — never put the EE inside the colliding funnel.
    drop_z = float(cone_height_m) + float(place_drop_clearance_m)
    place_drop = np.array(
        [place_xy[0], place_xy[1], drop_z], dtype=np.float64
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
            place_drop + np.array([0.0, 0.0, 0.10]),
            gripper_pinch,
        ),
        ("place_grasp", place_drop.copy(), gripper_pinch),
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
        if name == "pick_grasp":
            # Fresh sim: settle at kinematic grasp and require real pad contact.
            data_chk = mj.MjData(model)
            ok, pdist = _physics_pad_contact(
                model,
                data_chk,
                mj,
                seed=q,
                grip_val=gv,
                arm_act=arm_act,
                grip_act=grip_act,
                box_body=box_body,
            )
            if not ok:
                raise RuntimeError(
                    f"{name} physics pad contact missing "
                    f"(pad-mid dist={pdist * 1000:.1f} mm)"
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
        help="Ball radius in metres (SC-v14b default 0.043)",
    )
    args = parser.parse_args()
    wp = compute_waypoints(ball_radius_m=float(args.ball_radius_m))
    for key, val in wp.items():
        print(f"{key}: {val}")


if __name__ == "__main__":
    main()
