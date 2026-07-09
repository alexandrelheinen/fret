"""Unit tests for MuJoCo bridge core (T10-03).

Pure-Python tests — no ROS or MuJoCo runtime required for the core logic.
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from fret.ros.mujoco_bridge import (
    DubinsRaceBridgeCore,
    MuJoCoBridgeCore,
    cargo_weld_config_from_bridge_yaml,
    integrate_joint_velocities,
    make_dubins_race_bridge_core,
    make_mujoco_bridge_core,
    physics_config_from_bridge_yaml,
    resolve_mjcf_path,
)


def test_resolve_mjcf_ppp_warehouse() -> None:
    """MJCF path resolution should find the PPP warehouse preview scene."""
    path = resolve_mjcf_path("ppp", "ppp_warehouse", None)
    assert path.name == "ppp_warehouse.xml"
    assert path.is_file()


def test_resolve_mjcf_override_missing_raises() -> None:
    """Missing override path should raise FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        resolve_mjcf_path("ppp", "ppp_warehouse", "/nonexistent/scene.xml")


def test_resolve_mjcf_dubins_race() -> None:
    """MJCF path resolution should find the Dubins race scene."""
    path = resolve_mjcf_path("dubins", "dubins_race", None)
    assert path.name == "dubins_race.xml"
    assert path.is_file()


def test_resolve_mjcf_unsupported_model_raises() -> None:
    """Unsupported model/scenario pairs should raise ValueError."""
    with pytest.raises(ValueError, match="Unsupported"):
        resolve_mjcf_path("scara", "static_reach", None)


def test_integrate_joint_velocities_clips_limits() -> None:
    """Integration should clip positions to joint limits."""
    limits = np.array([[0.0, 1.0], [0.0, 2.0]], dtype=np.float64)
    q = np.array([0.5, 1.0])
    q_dot = np.array([10.0, 10.0])
    q_next = integrate_joint_velocities(q, q_dot, dt=1.0, limits=limits)
    np.testing.assert_array_equal(q_next, np.array([1.0, 2.0]))


def test_integrate_joint_velocities_shape_mismatch() -> None:
    """Mismatched shapes should raise ValueError."""
    limits = np.array([[0.0, 1.0], [0.0, 2.0]])
    with pytest.raises(ValueError):
        integrate_joint_velocities(
            np.array([0.0, 0.0]),
            np.array([1.0]),
            dt=0.02,
            limits=limits,
        )


def test_make_mujoco_bridge_core_ppp() -> None:
    """Factory should build a PPP bridge core with three joints."""
    core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    assert core.joint_names == ["joint_x", "joint_y", "joint_z"]
    assert core.get_positions().shape == (3,)
    assert core.mjcf_path.name == "ppp_warehouse.xml"


def test_make_mujoco_bridge_core_unknown_model() -> None:
    """Unknown models should raise ValueError."""
    with pytest.raises(ValueError, match="Unknown MuJoCo bridge model"):
        make_mujoco_bridge_core("scara", "static_reach")


def test_bridge_core_step_integrates_velocity() -> None:
    """Velocity commands should advance joint positions."""
    mjcf = resolve_mjcf_path("ppp", "ppp_warehouse", None)
    limits = np.array(
        [[0.0, 12.0], [0.0, 5.0], [0.0, 3.0]],
        dtype=np.float64,
    )
    core = MuJoCoBridgeCore(
        mjcf_path=mjcf,
        joint_names=["joint_x", "joint_y", "joint_z"],
        limits=limits,
        initial_positions=np.zeros(3),
    )
    q_next = core.step(np.array([1.0, 0.5, 0.2]), dt=0.02)
    np.testing.assert_allclose(q_next, np.array([0.02, 0.01, 0.004]))


def test_bridge_core_respects_joint_limits() -> None:
    """Repeated large velocity commands must not exceed MJCF limits."""
    core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    for _ in range(1000):
        core.step(np.array([10.0, 10.0, 10.0]), dt=0.02)
    q = core.get_positions()
    assert np.all(q >= core.limits[:, 0])
    assert np.all(q <= core.limits[:, 1])


def test_bridge_core_set_positions() -> None:
    """Direct position writes should be clipped to limits."""
    core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    core.set_positions(np.array([20.0, 6.0, 4.0]))
    np.testing.assert_array_equal(
        core.get_positions(),
        np.array([12.0, 5.0, 3.0]),
    )


def test_make_mujoco_bridge_core_dubins_raises() -> None:
    """Dubins race must use the dual-agent bridge factory."""
    with pytest.raises(ValueError, match="make_dubins_race_bridge_core"):
        make_mujoco_bridge_core("dubins", "dubins_race")


def test_dubins_race_bridge_core_pose_sync() -> None:
    """Dual-agent bridge should store RRT* and SST poses."""
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([2.0, 1.7, 0.5]),
        initial_sst=np.array([2.0, 2.3, 0.5]),
    )
    assert core.mjcf_path.name == "dubins_race.xml"
    np.testing.assert_allclose(core.get_rrt_pose(), [2.0, 1.7, 0.5], atol=1e-9)
    core.set_sst_pose((10.0, 8.0, 1.0))
    np.testing.assert_allclose(core.get_sst_pose()[:2], [10.0, 8.0], atol=1e-9)


def test_load_mujoco_config_defaults() -> None:
    """Default mujoco.yml should load PPP parameters."""
    from fret.ros.mujoco_bridge import (
        _load_bridge_config,
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    config_path = _resolve_config_path(None)
    cfg = _load_bridge_config(config_path)
    assert cfg["model"] == "ppp"
    assert cfg["scenario"] == "ppp_warehouse"
    assert cfg["update_rate"] == 50.0
    assert cfg["initial_joint_positions"] == [0.0, 0.0, 0.0]
    assert cfg["physics_mode"] is False
    assert cfg["substeps_per_tick"] == 25
    merged = _load_merged_bridge_config(config_path)
    assert "actuators" in merged
    assert pathlib.Path(config_path).is_file()


def test_physics_config_from_yaml_kinematic_default() -> None:
    """Kinematic mode should not require actuator runtime binding."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    physics = physics_config_from_bridge_yaml(cfg, "ppp")
    assert physics.physics_mode is False
    assert physics.actuators is None


def test_physics_config_from_yaml_physics_mode() -> None:
    """Physics mode should load the PPP actuator table."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "ppp", physics_mode=True)
    assert physics.physics_mode is True
    assert physics.actuators is not None
    assert physics.actuators.names == (
        "act_joint_x",
        "act_joint_y",
        "act_joint_z",
    )


@pytest.mark.skipif(
    not make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_bridge_core_step_physics_advances_position() -> None:
    """Physics mode should integrate joint motion via mj_step."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "ppp", physics_mode=True)
    core = make_mujoco_bridge_core(
        "ppp",
        "ppp_warehouse",
        physics_config=physics,
    )
    q0 = core.get_positions().copy()
    q1 = core.step_physics(np.array([0.5, 0.0, 0.0]))
    assert core.physics_mode is True
    assert q1[0] > q0[0]


@pytest.mark.skipif(
    not make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_set_positions_forbidden_in_physics_mode() -> None:
    """Pose injection must fail while physics_mode is active (FR-SIM-07)."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "ppp", physics_mode=True)
    core = make_mujoco_bridge_core(
        "ppp",
        "ppp_warehouse",
        physics_config=physics,
    )
    with pytest.raises(RuntimeError, match="set_positions"):
        core.set_positions(np.zeros(3))


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_dubins_bridge_step_physics_runs() -> None:
    """Dual-agent physics step should execute without pose injection."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([6.0, 6.0, 0.0]),
        initial_sst=np.array([6.0, 6.4, 0.0]),
        physics_config=physics,
    )
    result = core.step_physics(np.array([0.4, 0.0, 0.0, 0.4, 0.0, 0.0]))
    assert result.shape == (6,)
    np.testing.assert_allclose(
        core.get_joint_velocities(),
        [0.4, 0.0, 0.0, 0.4, 0.0, 0.0],
    )
    with pytest.raises(RuntimeError, match="set_rrt_pose"):
        core.set_rrt_pose((0.0, 0.0, 0.0))


def test_cargo_weld_config_from_yaml() -> None:
    """Merged mujoco config should expose PPP cargo weld settings."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    weld = cargo_weld_config_from_bridge_yaml(cfg)
    assert weld.equality_name == "cargo_weld"
    assert weld.body_parent == "z_hoist"
    assert weld.body_child == "cargo"


@pytest.mark.skipif(
    not make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_cargo_weld_toggle_kinematic_pose() -> None:
    """Kinematic mode should mirror cargo pose; physics mode toggles weld."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    weld_cfg = cargo_weld_config_from_bridge_yaml(cfg)
    core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    core.configure_cargo_weld(weld_cfg)

    box_anchor = np.array([2.0, 1.0, 0.25], dtype=np.float64)
    core.seed_cargo_pose(box_anchor)

    transport_pose = np.array([2.0, 1.0, 2.06], dtype=np.float64)
    core.sync_cargo_grasp(is_welded=True, cargo_pose=transport_pose)
    assert core._cargo_weld_active is False

    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "ppp", physics_mode=True)
    physics_core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    physics_core.configure_cargo_weld(weld_cfg)
    physics_core.seed_cargo_pose(box_anchor)
    physics_core.configure_physics(physics)
    physics_core.sync_cargo_grasp(is_welded=True, cargo_pose=transport_pose)
    assert physics_core._cargo_weld_active is True
    physics_core.sync_cargo_grasp(is_welded=False, cargo_pose=transport_pose)
    assert physics_core._cargo_weld_active is False
    with pytest.raises(RuntimeError, match="set_cargo_pose"):
        physics_core.set_cargo_pose(transport_pose)
