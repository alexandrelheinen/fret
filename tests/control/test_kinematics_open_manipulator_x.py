"""OpenMANIPULATOR-X kinematics + empty-cell A→B physics smoke (SC-v13a)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.kinematics import Kinematics
from fret.sitl_config import mjcf_path

mujoco = pytest.importorskip("mujoco")

# Matches config/scenarios/omx_reach.yml (green start → red goal).
_START = np.array([-0.8, -0.2, 0.3, 0.6], dtype=np.float64)
_GOAL = np.array([1.2, -0.4, 0.5, 0.4], dtype=np.float64)


def test_omx_kinematics_loads_menagerie() -> None:
    kin = Kinematics("open_manipulator_x")
    assert kin.dof == 4
    assert kin.joint_names == ["Joint1", "Joint2", "Joint3", "Joint4"]
    T = kin.forward_kinematics(_START)
    assert T.shape == (4, 4)
    assert np.isfinite(T).all()


def test_omx_tabletop_mjcf_resolves() -> None:
    path = mjcf_path("open_manipulator_x", "omx_reach")
    assert path.name == "omx_tabletop.xml"
    assert path.is_file()
    assert ".generated" in path.parts
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu >= 5


def test_omx_empty_cell_joint_space_a_to_b() -> None:
    """Drive Menagerie position actuators from start→goal; EE must move."""
    xml = mjcf_path("open_manipulator_x", "omx_reach")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    kin = Kinematics("open_manipulator_x")

    # Actuator order: Joint1..4, Gripper
    assert model.nu >= 5
    data.ctrl[:4] = _START
    data.ctrl[4] = 0.0
    for _ in range(200):
        mujoco.mj_step(model, data)

    ee0 = kin.forward_kinematics(_START)[:3, 3].copy()

    steps = 400
    for i in range(steps):
        alpha = (i + 1) / steps
        data.ctrl[:4] = (1.0 - alpha) * _START + alpha * _GOAL
        for _ in range(10):
            mujoco.mj_step(model, data)

    q_end = np.array(
        [
            data.qpos[model.jnt_qposadr[model.joint(n).id]]
            for n in kin.joint_names
        ],
        dtype=np.float64,
    )
    ee1 = kin.forward_kinematics(q_end)[:3, 3]
    err = float(np.linalg.norm(q_end - _GOAL))
    travel = float(np.linalg.norm(ee1 - ee0))

    assert err < 0.08, f"joint error {err:.3f} rad"
    assert travel > 0.05, f"EE barely moved ({travel:.3f} m)"
