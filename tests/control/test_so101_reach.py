"""SO-101 empty-cell A->B physics smoke test (SC-v20a, FR-SIM-20)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.kinematics import Kinematics
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = "src/fret/config/scenarios/so101_reach.yml"


def _scenario_start_goal() -> tuple[np.ndarray, np.ndarray]:
    params = load_scenario_parameters(_SCENARIO)
    start = np.asarray(params["start_configuration"], dtype=np.float64)
    goal = np.asarray(params["goal_configuration"], dtype=np.float64)
    return start, goal


def test_so101_tabletop_mjcf_resolves() -> None:
    path = mjcf_path("so101", "so101_reach")
    assert path.name == "so101_tabletop.xml"
    assert path.is_file()
    assert ".generated" in path.parts
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu == 6


def test_so101_empty_cell_joint_space_a_to_b_tracks_within_5mm() -> None:
    """Drive the position actuators start->goal; FR-SIM-20: <= 5 mm EE error."""
    xml = mjcf_path("so101", "so101_reach")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    kin = Kinematics("so101")
    start, goal = _scenario_start_goal()

    assert model.nu == 6
    data.ctrl[:5] = start
    data.ctrl[5] = 0.0  # gripper closed
    for _ in range(300):
        mujoco.mj_step(model, data)

    ee0 = kin.forward_kinematics(start)[:3, 3].copy()
    ee_goal = kin.forward_kinematics(goal)[:3, 3]
    planned_xy = float(np.linalg.norm(ee_goal[:2] - ee0[:2]))
    assert planned_xy > 0.40, f"SC-v20a path too short ({planned_xy:.3f} m XY)"

    steps = 800
    for i in range(steps):
        alpha = (i + 1) / steps
        data.ctrl[:5] = (1.0 - alpha) * start + alpha * goal
        for _ in range(10):
            mujoco.mj_step(model, data)
    # Settle at the goal.
    for _ in range(200):
        mujoco.mj_step(model, data)

    q_end = np.array(
        [
            data.qpos[model.jnt_qposadr[model.joint(n).id]]
            for n in kin.joint_names
        ],
        dtype=np.float64,
    )
    ee1 = kin.forward_kinematics(q_end)[:3, 3]
    ee_err = float(np.linalg.norm(ee1 - ee_goal))

    assert ee_err < 0.005, f"FR-SIM-20: EE error {ee_err * 1000:.2f} mm > 5 mm"
