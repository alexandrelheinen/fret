"""Regression: obstacles must physically collide with the full arm.

Prior agents left the vision gate on ``contype=0`` (ghost) and scene walls /
place-bin shells on ``contype=1`` while remapping pads/wrist to bit 4. MuJoCo's
contact filter then allowed the gripper to occupy the same volume as a solid
brown Γ wall or gray portal with **zero** pad–obstacle contacts — exactly the
clipping in the maintainer screenshots.

Obstacles use ``contype=5`` (bits 1|4) so proximal **and** distal collide.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.sitl_config import mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENES = (
    ("open_manipulator_x", "omx_wall_maze"),
    ("open_manipulator_x", "omx_desk_clutter"),
    ("open_manipulator_x", "omx_pick_place"),
    ("open_manipulator_y", "omy_clutter"),
    ("open_manipulator_y", "omy_pick_place"),
)

_FULL_ARM_BITS = 5  # proximal=1 | distal/pads=4
_GATE_STRUCTURAL = (
    "gate_post_l",
    "gate_post_r",
    "gate_beam_top",
    "gate_beam_bot",
    "gate_brace_a",
    "gate_brace_b",
)


def _filter_allows(model: object, i: int, j: int) -> bool:
    return bool(
        (int(model.geom_contype[i]) & int(model.geom_conaffinity[j]))
        and (int(model.geom_contype[j]) & int(model.geom_conaffinity[i]))
    )


@pytest.mark.parametrize(("robot", "scene"), _SCENES)
def test_obstacles_use_full_arm_contact_bits(robot: str, scene: str) -> None:
    path = mjcf_path(robot, scene)
    model = mujoco.MjModel.from_xml_path(str(path))
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if name.startswith("place_bin") or name.startswith("transfer_wall"):
            assert int(model.geom_contype[i]) == _FULL_ARM_BITS, name
            assert int(model.geom_conaffinity[i]) == _FULL_ARM_BITS, name
        if name in _GATE_STRUCTURAL:
            assert int(model.geom_contype[i]) == _FULL_ARM_BITS, name
            assert int(model.geom_conaffinity[i]) == _FULL_ARM_BITS, name


def test_pad_collides_with_gamma_wall_and_gate() -> None:
    """Pad↔Γ wall and proximal↔gate filters must be live (not silent ghosts)."""
    path = mjcf_path("open_manipulator_x", "omx_wall_maze")
    model = mujoco.MjModel.from_xml_path(str(path))
    pad = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pad_left")
    stem = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "transfer_wall_stem"
    )
    gate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "gate_beam_bot")
    assert pad >= 0 and stem >= 0 and gate >= 0
    assert _filter_allows(model, pad, stem)
    assert _filter_allows(model, pad, gate)

    # Geometric penetration of pad into the stem must produce a pad–stem contact.
    data = mujoco.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    qadrs = [
        int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            ]
        )
        for n in names
    ]
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    data.qpos[int(model.jnt_qposadr[box_jid]) : int(model.jnt_qposadr[box_jid]) + 3] = [
        0.5,
        0.5,
        0.5,
    ]
    data.qpos[qadrs[0]] = -0.40
    data.qpos[qadrs[1]] = 0.75
    data.qpos[qadrs[2]] = -1.20
    data.qpos[qadrs[3]] = 0.50
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    fromto = np.empty(6)
    dist = float(
        mujoco.mj_geomDistance(model, data, pad, stem, 10.0, fromto)
    )
    assert dist < 0.0, f"expected pad–stem penetration, got dist={dist}"
    pad_stem_hits = 0
    for ci in range(data.ncon):
        c = data.contact[ci]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or ""
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or ""
        pair = {g1, g2}
        if "pad_left" in pair and "transfer_wall_stem" in pair:
            pad_stem_hits += 1
    assert pad_stem_hits >= 1, (
        f"pad penetrated stem (dist={dist:.4f}) but no pad–stem contact "
        f"(ncon={data.ncon}) — contact filter still wrong"
    )


def test_omy_distal_collides_with_transfer_wall() -> None:
    path = mjcf_path("open_manipulator_y", "omy_clutter")
    model = mujoco.MjModel.from_xml_path(str(path))
    pad = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pad_left")
    wall = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "transfer_wall_a"
    )
    gate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "gate_brace_a")
    assert pad >= 0 and wall >= 0 and gate >= 0
    assert _filter_allows(model, pad, wall)
    assert _filter_allows(model, pad, gate)
