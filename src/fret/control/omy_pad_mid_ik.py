"""Pad-midpoint numerical IK for OpenMANIPULATOR-Y fingertip pads."""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt
from scipy.optimize import minimize

OMY_ARM_JOINTS: tuple[str, ...] = (
    "Joint1",
    "Joint2",
    "Joint3",
    "Joint4",
    "Joint5",
    "Joint6",
)
OMY_GRIPPER_PINCH: float = 1.05


def pad_mid_ik(
    model: Any,
    data: Any,
    mj: Any,
    *,
    target: npt.NDArray[np.floating[Any]],
    grip_val: float,
    seed: npt.NDArray[np.floating[Any]],
    limits: npt.NDArray[np.floating[Any]],
    pad_right: int,
    pad_left: int,
    grip_adr: int,
    arm_joints: tuple[str, ...] = OMY_ARM_JOINTS,
) -> tuple[npt.NDArray[np.float64], float]:
    """Return joint vector placing pad-mid at ``target`` (kinematic FK)."""
    target_arr = np.asarray(target, dtype=np.float64)

    def set_kin(q: npt.NDArray[np.floating[Any]]) -> None:
        for i, name in enumerate(arm_joints):
            adr = int(
                model.jnt_qposadr[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                ]
            )
            data.qpos[adr] = float(q[i])
        data.qpos[grip_adr] = float(grip_val)
        mj.mj_forward(model, data)

    def midpoint() -> npt.NDArray[np.float64]:
        return 0.5 * (
            np.asarray(data.geom_xpos[pad_right], dtype=np.float64)
            + np.asarray(data.geom_xpos[pad_left], dtype=np.float64)
        )

    def cost(q: npt.NDArray[np.floating[Any]]) -> float:
        set_kin(q)
        return float(np.sum((midpoint() - target_arr) ** 2))

    res = minimize(
        cost,
        np.asarray(seed, dtype=np.float64),
        method="L-BFGS-B",
        bounds=limits,
        options={"maxiter": 500, "ftol": 1e-14},
    )
    set_kin(res.x)
    err = float(np.linalg.norm(midpoint() - target_arr))
    return res.x.astype(np.float64), err
