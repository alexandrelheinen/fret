"""OpenMANIPULATOR-X kinematics + empty-cell A→B physics smoke (SC-v13a)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.kinematics import Kinematics
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = "src/fret/config/scenarios/omx_reach.yml"
# Hard Menagerie limits; demos keep 10° off each stop (see robots/open_manipulator_x.md).
_YAW_HARD_DEG = 180.0
_YAW_MARGIN_DEG = 10.0
_YAW_USABLE_DEG = _YAW_HARD_DEG - _YAW_MARGIN_DEG
# SC-v13a uses a 160° (±80°) yaw slew — long free-space arc with headroom.
_DEMO_YAW_SPAN_DEG = 160.0


def _scenario_start_goal() -> tuple[np.ndarray, np.ndarray]:
    params = load_scenario_parameters(_SCENARIO)
    start = np.asarray(params["start_configuration"], dtype=np.float64)
    goal = np.asarray(params["goal_configuration"], dtype=np.float64)
    return start, goal


def test_omx_kinematics_loads_menagerie() -> None:
    kin = Kinematics("open_manipulator_x")
    assert kin.dof == 4
    assert kin.joint_names == ["Joint1", "Joint2", "Joint3", "Joint4"]
    start, _ = _scenario_start_goal()
    T = kin.forward_kinematics(start)
    assert T.shape == (4, 4)
    assert np.isfinite(T).all()


def test_omx_joint1_yaw_limits_and_demo_margin() -> None:
    """Joint1 is ±π; SC-v13a stays inside the 10° planning margin."""
    kin = Kinematics("open_manipulator_x")
    j1_lo, j1_hi = kin.joint_limits[0]
    assert j1_lo == pytest.approx(-np.pi, abs=1e-4)
    assert j1_hi == pytest.approx(np.pi, abs=1e-4)

    start, goal = _scenario_start_goal()
    yaw_span_deg = float(np.rad2deg(abs(goal[0] - start[0])))
    assert yaw_span_deg == pytest.approx(_DEMO_YAW_SPAN_DEG, abs=0.5)
    for q in (start, goal):
        yaw_deg = abs(float(np.rad2deg(q[0])))
        assert yaw_deg <= _YAW_USABLE_DEG + 1e-6
        assert yaw_deg <= _YAW_HARD_DEG - _YAW_MARGIN_DEG + 1e-6


def test_omx_tabletop_mjcf_resolves() -> None:
    path = mjcf_path("open_manipulator_x", "omx_reach")
    assert path.name == "omx_tabletop.xml"
    assert path.is_file()
    assert ".generated" in path.parts
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu >= 5


def test_omx_empty_cell_joint_space_a_to_b() -> None:
    """Drive Menagerie position actuators start→goal; EE must travel far."""
    xml = mjcf_path("open_manipulator_x", "omx_reach")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    kin = Kinematics("open_manipulator_x")
    start, goal = _scenario_start_goal()

    assert model.nu >= 5
    data.ctrl[:4] = start
    data.ctrl[4] = 0.0
    for _ in range(300):
        mujoco.mj_step(model, data)

    ee0 = kin.forward_kinematics(start)[:3, 3].copy()
    ee_goal = kin.forward_kinematics(goal)[:3, 3]
    planned_xy = float(np.linalg.norm(ee_goal[:2] - ee0[:2]))
    assert planned_xy > 0.40, f"SC-v13a path too short ({planned_xy:.3f} m XY)"

    steps = 800
    for i in range(steps):
        alpha = (i + 1) / steps
        data.ctrl[:4] = (1.0 - alpha) * start + alpha * goal
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
    err = float(np.linalg.norm(q_end - goal))
    travel_xy = float(np.linalg.norm(ee1[:2] - ee0[:2]))

    assert err < 0.08, f"joint error {err:.3f} rad"
    assert travel_xy > 0.40, f"EE XY travel too small ({travel_xy:.3f} m)"
