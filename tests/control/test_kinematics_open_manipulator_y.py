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
    from fret.control.omy_pick_place_sim import simulate_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    state, samples = simulate_omy_pick_place(
        duration_s=180.0,
        joint_tol_rad=0.22,
        scenario_path=_PICK,
    )
    assert any(s.state == PickPlaceState.GRASP for s in samples)
    assert any(s.state == PickPlaceState.LIFT for s in samples)


@pytest.mark.slow
def test_omy_pick_place_physics_moves_ball_into_place_cone() -> None:
    """Idle→pinch→hold→place→idle under physics (OMX honesty bar)."""
    from fret.control.omy_pick_place_sim import (
        run_omy_pick_place,
        waypoints_from_scenario,
    )
    from fret.control.pick_place_fsm import PickPlaceState

    params = load_scenario_parameters(_PICK)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    cone_r = float(params.get("place_cone_radius_m", 0.14))
    wp = waypoints_from_scenario(_PICK)
    state, ball = run_omy_pick_place(
        duration_s=45.0,
        joint_tol_rad=0.16,
        scenario_path=_PICK,
    )
    assert state != PickPlaceState.FAULT, "OMY pick-place FSM faulted"
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    assert float(np.linalg.norm(ball[:2] - place_xy)) < cone_r
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    assert float(ball[2]) < 0.08
    # Retreat holds the distinct idle fold (not hover/retreat alias).
    assert float(np.linalg.norm(wp.idle - wp.pick_hover)) > 0.3
    assert np.allclose(wp.retreat, wp.idle)


def test_omy_clutter_transfer_detour_is_collision_free() -> None:
    """Release detour must clear the mid-cell wall (v1.2.5 regression)."""
    from fret.control.pick_place_planning import walls_from_scenario
    from fret.control.pick_place_sim import waypoints_from_scenario
    from fret.planning.cspace_checker import make_cspace_checker
    from fret.scene.occupancy_adapter import OccupancyAdapter
    from fret.interfaces import OccupancyUpdatePayload

    params = load_scenario_parameters(_CLUTTER)
    wp = waypoints_from_scenario(_CLUTTER)
    mid = np.asarray(params["transfer_detour_configuration"], dtype=np.float64)
    detour = [wp.lift_hover, mid, wp.place_hover]

    walls = walls_from_scenario(_CLUTTER, inflate=False)
    kin = Kinematics("omy")
    rng = np.random.default_rng(int(params.get("planner_rng_seed", 3)))
    from fret.control.pick_place_planning import _sample_walls

    pts = _sample_walls(walls, float(params["occupancy_density"]), rng)
    adapter = OccupancyAdapter()
    adapter.update(
        OccupancyUpdatePayload(
            obstacle_points=pts, timestamp=0.0, frame_id="world"
        )
    )
    checker = make_cspace_checker(kin, adapter.get_occupancy())
    for i, q in enumerate(detour):
        assert checker.is_collision_free(q), f"detour waypoint {i} collides"
    for i in range(len(detour) - 1):
        edge_mid = 0.5 * (detour[i] + detour[i + 1])
        assert checker.is_collision_free(edge_mid), f"detour segment {i} collides"


@pytest.mark.slow
def test_omy_pick_place_no_ball_teleport() -> None:
    """Ball position must evolve continuously — no kinematic pad latch."""
    from fret.control.omy_pick_place_sim import simulate_omy_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    state, samples = simulate_omy_pick_place(
        duration_s=180.0,
        joint_tol_rad=0.22,
        scenario_path=_PICK,
    )
    assert samples, "expected recorded samples"
    max_step_jump_m = 0.0
    for prev, cur in zip(samples, samples[1:]):
        jump = float(np.linalg.norm(cur.box_qpos[:3] - prev.box_qpos[:3]))
        max_step_jump_m = max(max_step_jump_m, jump)
    # Physics + 2 ms timestep: sub-centimetre per recorded sample.
    assert max_step_jump_m < 0.05, f"ball teleported {max_step_jump_m:.3f} m"


@pytest.mark.slow
def test_omy_clutter_pick_place_smoke() -> None:
    from fret.control.omy_clutter_sim import run_omy_clutter_pick_place
    from fret.control.pick_place_fsm import PickPlaceState

    params = load_scenario_parameters(_CLUTTER)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    result = run_omy_clutter_pick_place(
        duration_s=180.0,
        joint_tol_rad=0.45,
        scenario_path=_CLUTTER,
        seed_offset=0,
    )
    assert any(
        s.state
        in {
            PickPlaceState.DESCEND_PICK,
            PickPlaceState.GRASP,
            PickPlaceState.LIFT,
        }
        for s in result.samples
    )
    assert result.state != PickPlaceState.FAULT or any(
        s.state == PickPlaceState.LIFT for s in result.samples
    )
    assert result.straight_line_collides
    assert len(result.transfer_path) >= 2
    if result.state == PickPlaceState.DONE:
        assert float(np.linalg.norm(result.box_pos[:2] - place_xy)) < 0.10