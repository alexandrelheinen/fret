"""Unit tests for MuJoCo bridge core (T10-03).

Pure-Python tests — no ROS or MuJoCo runtime required for the core logic.
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from fret.ros.mujoco_bridge import (
    MuJoCoBridgeCore,
    integrate_joint_velocities,
    make_mujoco_bridge_core,
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


def test_resolve_mjcf_unsupported_model_raises() -> None:
    """Unsupported model/scenario pairs should raise ValueError."""
    with pytest.raises(ValueError, match="Unsupported"):
        resolve_mjcf_path("dubins", "dubins_race", None)


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
        [[0.0, 12.0], [0.0, 4.0], [0.0, 3.0]],
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
    core.set_positions(np.array([20.0, 5.0, 4.0]))
    np.testing.assert_array_equal(
        core.get_positions(),
        np.array([12.0, 4.0, 3.0]),
    )


def test_load_mujoco_config_defaults() -> None:
    """Default mujoco.yml should load PPP parameters."""
    from fret.ros.mujoco_bridge import (
        _load_bridge_config,
        _resolve_config_path,
    )

    config_path = _resolve_config_path(None)
    cfg = _load_bridge_config(config_path)
    assert cfg["model"] == "ppp"
    assert cfg["scenario"] == "ppp_warehouse"
    assert cfg["update_rate"] == 50.0
    assert cfg["initial_joint_positions"] == [0.0, 0.0, 0.0]
    assert pathlib.Path(config_path).is_file()
