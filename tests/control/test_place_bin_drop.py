"""The ball must reach the bin floor in every place-bin scenario.

Regression guard for the invisible ``funnel_w*`` catcher cone: it collided
with the ball (bit 2 against the ball's 2|4|8 mask) while rendering fully
transparent, so a placed ball came to rest 22 mm (OM-X) or 50 mm (OMY)
above the bin floor, suspended on geometry no viewer could see.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.sitl_config import mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENES = [
    ("open_manipulator_x", "omx_pick_place"),
    ("open_manipulator_x", "omx_desk_clutter"),
    ("open_manipulator_x", "omx_wall_maze"),
    ("omy", "omy_pick_place"),
    ("omy", "omy_clutter"),
]


@pytest.mark.parametrize("robot_model,scenario_id", _SCENES)
def test_ball_dropped_into_the_bin_rests_on_its_floor(
    robot_model: str, scenario_id: str
) -> None:
    model = mujoco.MjModel.from_xml_path(
        str(mjcf_path(robot_model, scenario_id))
    )
    data = mujoco.MjData(model)

    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pick_box_geom")
    bottom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "place_bin_bottom"
    )
    radius = float(model.geom_size[ball][0])
    floor_z = float(model.geom_pos[bottom][2] + model.geom_size[bottom][2])

    joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    adr = int(model.jnt_qposadr[joint])
    data.qpos[adr : adr + 3] = [
        float(model.geom_pos[bottom][0]),
        float(model.geom_pos[bottom][1]),
        floor_z + 0.09,
    ]
    data.qpos[adr + 3 : adr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0

    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    rest_z = float(data.qpos[adr + 2])
    assert rest_z == pytest.approx(floor_z + radius, abs=2e-3), (
        f"{scenario_id}: ball rests at {rest_z:.4f} m, "
        f"expected {floor_z + radius:.4f} m"
    )

    resting_on = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
        for contact in data.contact[: data.ncon]
        for geom in (contact.geom1, contact.geom2)
    } - {"pick_box_geom"}
    assert (
        "place_bin_bottom" in resting_on
    ), f"{scenario_id}: ball rests on {sorted(resting_on)}"
