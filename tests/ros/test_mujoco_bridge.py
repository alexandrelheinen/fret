"""Unit tests for MuJoCo bridge core (T10-03).

Pure-Python tests — no ROS or MuJoCo runtime required for the core logic.
"""

from __future__ import annotations

import pathlib

import numpy as np
import numpy.typing as npt
import pytest

from fret.ros.mujoco_bridge import (
    MuJoCoBridgeCore,
    integrate_joint_velocities,
    make_dubins_race_bridge_core,
    make_mujoco_bridge_core,
    physics_config_from_bridge_yaml,
    resolve_mjcf_path,
)

_DUBINS_LIMITS = np.array(
    [
        [0.0, 10.0],
        [0.0, 10.0],
        [-np.pi, np.pi],
    ],
    dtype=np.float64,
)

# Minimal SE(2) scene for MuJoCoBridgeCore kinematic tests (race MJCF is freejoint).
_MINIMAL_SE2_MJCF = """\
<mujoco model="se2_kinematic_test">
  <worldbody>
    <body name="agent" pos="0 0 0">
      <joint name="joint_x" type="slide" axis="1 0 0"/>
      <joint name="joint_y" type="slide" axis="0 1 0"/>
      <joint name="joint_yaw" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.05" rgba="0.2 0.4 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture()
def se2_mjcf_path(tmp_path: pathlib.Path) -> pathlib.Path:
    """Write a tiny three-slide MJCF for generic bridge-core tests."""
    path = tmp_path / "se2_kinematic_test.xml"
    path.write_text(_MINIMAL_SE2_MJCF, encoding="utf-8")
    return path


def _single_agent_core(
    mjcf_path: pathlib.Path,
    *,
    initial: npt.NDArray[np.float64] | None = None,
) -> MuJoCoBridgeCore:
    """Build a kinematic MuJoCoBridgeCore on a minimal SE(2) MJCF."""
    q0 = (
        np.zeros(3, dtype=np.float64)
        if initial is None
        else np.asarray(initial, dtype=np.float64)
    )
    return MuJoCoBridgeCore(
        mjcf_path=mjcf_path,
        joint_names=["joint_x", "joint_y", "joint_yaw"],
        limits=_DUBINS_LIMITS,
        initial_positions=q0,
    )


def test_resolve_mjcf_override_missing_raises() -> None:
    """Missing override path should raise FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        resolve_mjcf_path("dubins", "dubins_race", "/nonexistent/scene.xml")


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


def test_make_mujoco_bridge_core_unknown_model() -> None:
    """Unknown models should raise ValueError."""
    with pytest.raises(ValueError, match="Unknown MuJoCo bridge model"):
        make_mujoco_bridge_core("scara", "static_reach")


def test_bridge_core_step_integrates_velocity(
    se2_mjcf_path: pathlib.Path,
) -> None:
    """Velocity commands should advance joint positions."""
    core = _single_agent_core(se2_mjcf_path)
    q_next = core.step(np.array([1.0, 0.5, 0.2]), dt=0.02)
    np.testing.assert_allclose(q_next, np.array([0.02, 0.01, 0.004]))


def test_bridge_core_respects_joint_limits(
    se2_mjcf_path: pathlib.Path,
) -> None:
    """Repeated large velocity commands must not exceed MJCF limits."""
    core = _single_agent_core(se2_mjcf_path)
    for _ in range(1000):
        core.step(np.array([10.0, 10.0, 10.0]), dt=0.02)
    q = core.get_positions()
    assert np.all(q >= core.limits[:, 0])
    assert np.all(q <= core.limits[:, 1])


def test_bridge_core_set_positions(se2_mjcf_path: pathlib.Path) -> None:
    """Direct position writes should be clipped to limits."""
    core = _single_agent_core(se2_mjcf_path)
    core.set_positions(np.array([200.0, 200.0, 10.0]))
    np.testing.assert_array_equal(
        core.get_positions(),
        np.array([10.0, 10.0, np.pi]),
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
    """Default mujoco.yml should load Dubins parameters."""
    from fret.ros.mujoco_bridge import (
        _load_bridge_config,
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    config_path = _resolve_config_path(None)
    cfg = _load_bridge_config(config_path)
    assert cfg["model"] == "dubins"
    assert cfg["scenario"] == "dubins_race"
    assert cfg["update_rate"] == 50.0
    assert cfg["initial_joint_positions"] == [0.0, 0.0, 0.0]
    assert cfg["physics_mode"] is False
    assert cfg["substeps_per_tick"] == 40
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
    physics = physics_config_from_bridge_yaml(cfg, "dubins")
    assert physics.physics_mode is False
    assert physics.actuators is None


def test_physics_config_from_yaml_physics_mode() -> None:
    """Physics mode should load the Dubins actuator table."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    assert physics.physics_mode is True
    assert physics.actuators is not None
    assert physics.actuators.names[0] == "rrt_wheel_left"
    assert physics.actuators.names == (
        "rrt_wheel_left",
        "rrt_wheel_right",
        "sst_wheel_left",
        "sst_wheel_right",
        "dummy_wheel_left",
        "dummy_wheel_right",
    )
    assert len(physics.actuators.names) == 6


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
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
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([1.2, 1.2, 0.0]),
        initial_sst=np.array([1.2, 1.5, 0.0]),
        physics_config=physics,
    )
    q0 = core.get_rrt_pose().copy()
    core.step_physics(np.array([0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert core.physics_mode is True
    assert core.get_rrt_pose()[0] > q0[0]


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
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
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(physics_config=physics)
    with pytest.raises(RuntimeError, match="set_rrt_pose"):
        core.set_rrt_pose((0.0, 0.0, 0.0))


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_get_collision_forces_n_zero_in_open_space() -> None:
    """No obstacle nearby: peak contact force must stay zero (v1.3 monitor)."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([1.2, 1.2, 0.0]),
        initial_sst=np.array([1.2, 1.5, 0.0]),
        physics_config=physics,
    )
    for _ in range(10):
        core.step_physics(
            np.array([0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )
    rrt_force, sst_force, dummy_force = core.get_collision_forces_n()
    assert rrt_force == 0.0
    assert sst_force == 0.0
    assert dummy_force == 0.0


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_get_collision_forces_n_detects_real_structure_contact() -> None:
    """Driving straight into a structure geom must report a real contact
    force (v1.3 collision monitor's primary signal, replacing the removed
    pre-emptive occupancy-based motion block)."""
    from fret.ros.mujoco_bridge import (
        _load_merged_bridge_config,
        _resolve_config_path,
    )

    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    # str_000_col sits at (2.491, 3.509) with half-extent (0.55, 0.15) in
    # dubins_race.xml, so its near face is at x=1.941. Spawn the RRT agent
    # 0.44 m away, facing it, and drive straight in.
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([1.5, 3.509, 0.0]),
        initial_sst=np.array([1.2, 1.5, 0.0]),
        physics_config=physics,
    )
    rrt_force = 0.0
    for _ in range(40):
        core.step_physics(
            np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )
        rrt_force, _sst_force, _dummy_force = core.get_collision_forces_n()
        if rrt_force > 0.0:
            break
    assert rrt_force > 0.0


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
        initial_rrt=np.array([1.2, 1.2, 0.0]),
        initial_sst=np.array([1.2, 1.5, 0.0]),
        physics_config=physics,
    )
    result = core.step_physics(
        np.array([0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0])
    )
    assert result.shape == (9,)
    np.testing.assert_allclose(
        core.get_joint_velocities(),
        [0.15, 0.0, 0.0, 0.15, 0.0, 0.0, 0.15, 0.0, 0.0],
    )
    assert core.get_rrt_pose()[0] > 1.2
    assert core.get_sst_pose()[0] > 1.2
    with pytest.raises(RuntimeError, match="set_rrt_pose"):
        core.set_rrt_pose((0.0, 0.0, 0.0))
