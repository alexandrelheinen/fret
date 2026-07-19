"""OpenMANIPULATOR-Y kinematics + scenario smoke tests (SC-v14)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.kinematics import Kinematics
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_REACH = "src/fret/config/scenarios/omy_reach.yml"
_PICK = "src/fret/config/scenarios/omy_pick_place.yml"
_CLUTTER = "src/fret/config/scenarios/omy_clutter.yml"


def _scenario_start_goal(path: str) -> tuple[np.ndarray, np.ndarray]:
    params = load_scenario_parameters(path)
    start = np.asarray(params["start_configuration"], dtype=np.float64)
    goal = np.asarray(params["goal_configuration"], dtype=np.float64)
    return start, goal


def test_omy_kinematics_loads_menagerie() -> None:
    kin = Kinematics("omy")
    assert kin.dof == 6
    assert len(kin.joint_names) == 6
    start, _ = _scenario_start_goal(_REACH)
    T = kin.forward_kinematics(start)
    assert T.shape == (4, 4)
    assert np.isfinite(T).all()


def test_omy_tabletop_mjcf_resolves() -> None:
    path = mjcf_path("omy", "omy_reach")
    assert path.name == "omy_tabletop.xml"
    assert path.is_file()
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu >= 7


def test_omy_empty_cell_joint_space_a_to_b() -> None:
    xml = mjcf_path("omy", "omy_reach")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    kin = Kinematics("omy")
    start, goal = _scenario_start_goal(_REACH)

    for i in range(6):
        data.ctrl[i] = float(start[i])
    data.ctrl[6] = 0.0
    for _ in range(300):
        mujoco.mj_step(model, data)

    ee0 = kin.forward_kinematics(start)[:3, 3].copy()
    ee_goal = kin.forward_kinematics(goal)[:3, 3]
    planned_xy = float(np.linalg.norm(ee_goal[:2] - ee0[:2]))
    assert planned_xy > 0.5

    steps = 1000
    for i in range(steps):
        alpha = (i + 1) / steps
        for j in range(6):
            data.ctrl[j] = (1.0 - alpha) * start[j] + alpha * goal[j]
        for _ in range(10):
            mujoco.mj_step(model, data)

    q_end = np.array(
        [
            data.qpos[model.jnt_qposadr[model.joint(n).id]]
            for n in kin.joint_names
        ],
        dtype=np.float64,
    )
    err = float(np.linalg.norm(q_end - goal))
    assert err < 0.35


@pytest.mark.slow
def test_omy_pick_place_physics_smoke() -> None:
    from fret.control.omy_pick_place_sim import run_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    state, box_pos = run_omy_pick_place(
        duration_s=55.0,
        joint_tol_rad=0.22,
        scenario_path=_PICK,
    )
    assert state == PickPlaceState.DONE
    assert float(box_pos[2]) < 0.08


@pytest.mark.slow
def test_omy_clutter_pick_place_smoke() -> None:
    from fret.control.omy_clutter_sim import run_omy_clutter_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    result = run_omy_clutter_pick_place(
        duration_s=90.0,
        joint_tol_rad=0.28,
        scenario_path=_CLUTTER,
        seed_offset=0,
    )
    assert result.state == PickPlaceState.DONE
    assert result.straight_line_collides
    assert len(result.transfer_path) >= 2