"""Unit tests for PPP prismatic velocity controller (T10-09)."""

from __future__ import annotations

import pathlib

import numpy as np
import pytest
import yaml

from fret.control.controller_node import make_controller_node
from fret.control.controller_ppp import PPPControllerNode, PPPControllerState
from fret.control.kinematics import Kinematics


def _ppp_config_path() -> pathlib.Path:
    return (
        pathlib.Path(__file__).resolve().parents[2]
        / "src"
        / "fret"
        / "config"
        / "controllers"
        / "ppp.yml"
    )


def test_construction() -> None:
    """PPPControllerNode can be constructed with a config path."""
    node = PPPControllerNode(str(_ppp_config_path()))
    assert node is not None
    assert node.update_rate == 50.0


def test_initial_command_is_zero() -> None:
    """Command vector must be zero on construction."""
    node = PPPControllerNode(str(_ppp_config_path()))
    cmd = node._get_current_command()
    np.testing.assert_array_equal(cmd, np.zeros(3))


def test_fault_threshold_10mm() -> None:
    """PPP fault threshold is 10 mm (FR-CTL-06)."""
    node = PPPControllerNode(str(_ppp_config_path()))
    assert node.fault_threshold == 0.010
    assert node._check_fault(0.011) is True
    assert node._check_fault(0.010) is False


def test_velocity_clipping() -> None:
    """Large joint error must be clipped to per-axis max velocity."""
    config = {
        "/**": {
            "ros__parameters": {
                "kp": 10.0,
                "max_joint_velocity": [3.0, 3.0, 1.5],
                "fault_threshold": 25.0,
                "ticks_per_waypoint": 1,
            }
        }
    }
    path = pathlib.Path("/tmp") / "ppp_clip.yml"
    path.write_text(yaml.dump(config))
    node = PPPControllerNode(str(path))
    kin = Kinematics("ppp")
    q_far = np.array([10.0, 10.0, 10.0])
    node.set_trajectory([np.zeros(3), q_far])
    node._trajectory_index = 1
    q_dot = node.compute_prismatic_command(kin, np.zeros(3))
    np.testing.assert_array_equal(q_dot, np.array([3.0, 3.0, 1.5]))


def test_tracking_reaches_target_within_10mm(tmp_path: pathlib.Path) -> None:
    """P-control should drive error below 10 mm within 2 s at 50 Hz."""
    config_data = {
        "/**": {
            "ros__parameters": {
                "kp": 1.5,
                "max_joint_velocity": [3.0, 3.0, 1.5],
                "fault_threshold": 0.010,
                "update_rate": 50.0,
                "ticks_per_waypoint": 10_000,
            }
        }
    }
    config_file = tmp_path / "ppp.yml"
    config_file.write_text(yaml.dump(config_data))
    node = PPPControllerNode(str(config_file))
    kin = Kinematics("ppp")
    q_start = np.array([0.495, 0.295, 0.195])
    q_goal = np.array([0.5, 0.3, 0.2])
    node.set_trajectory([q_start.copy(), q_goal.copy()])
    node._trajectory_index = 1

    dt = 1.0 / node.update_rate
    q = q_start.copy()
    for _ in range(int(2.0 / dt)):
        q_dot = node.compute_prismatic_command(kin, q)
        q = q + q_dot * dt

    assert node.state == PPPControllerState.TRACKING
    ee_error = float(np.linalg.norm(q - q_goal))
    assert ee_error < 0.010


def test_fault_on_unreachable_target() -> None:
    """Controller should halt when tracking error exceeds fault threshold."""
    config = {
        "/**": {
            "ros__parameters": {
                "kp": 0.1,
                "max_joint_velocity": [3.0, 3.0, 1.5],
                "fault_threshold": 0.010,
                "ticks_per_waypoint": 1,
            }
        }
    }
    path = pathlib.Path("/tmp") / "ppp_fault.yml"
    path.write_text(yaml.dump(config))
    node = PPPControllerNode(str(path))
    kin = Kinematics("ppp")
    node.set_trajectory([np.zeros(3), np.array([1.0, 0.0, 0.0])])

    dt = 1.0 / node.update_rate
    q = np.zeros(3)
    for _ in range(20):
        q_dot = node.compute_prismatic_command(kin, q)
        q = q + q_dot * dt

    assert node.state == PPPControllerState.HALTED
    np.testing.assert_array_equal(node._get_current_command(), np.zeros(3))


def test_halted_outputs_zero_velocity() -> None:
    """Halted controller should output zero velocity."""
    node = PPPControllerNode(str(_ppp_config_path()))
    kin = Kinematics("ppp")
    node._enter_halted()
    q_dot = node.compute_prismatic_command(kin, np.zeros(3))
    np.testing.assert_array_equal(q_dot, np.zeros(3))


def test_set_trajectory_requires_two_waypoints() -> None:
    """Trajectory must contain at least two waypoints."""
    node = PPPControllerNode(str(_ppp_config_path()))
    with pytest.raises(ValueError, match="at least 2 waypoints"):
        node.set_trajectory([np.zeros(3)])


def test_config_loading_from_yaml(tmp_path: pathlib.Path) -> None:
    """Custom threshold from YAML must override the default."""
    config_file = tmp_path / "ppp.yml"
    config_data = {
        "/**": {
            "ros__parameters": {
                "fault_threshold": 0.050,
                "kp": 2.0,
                "max_joint_velocity": [1.0, 2.0, 3.0],
                "update_rate": 100.0,
            }
        }
    }
    config_file.write_text(yaml.dump(config_data))
    node = PPPControllerNode(str(config_file))
    assert node.fault_threshold == 0.050
    assert node.update_rate == 100.0
    assert node._check_fault(0.040) is False
    assert node._check_fault(0.060) is True


def test_make_controller_node_ppp() -> None:
    """Factory should dispatch to PPPControllerNode for model='ppp'."""
    node = make_controller_node("ppp", _ppp_config_path())
    assert isinstance(node, PPPControllerNode)
    assert node.fault_threshold == 0.010


def test_make_controller_node_scara(tmp_path: pathlib.Path) -> None:
    """Factory should dispatch to ControllerNode for model='scara'."""
    from fret.control.controller_node import ControllerNode

    node = make_controller_node("scara", tmp_path)
    assert isinstance(node, ControllerNode)


def test_make_controller_node_unknown_model() -> None:
    """Factory should raise ValueError for unknown models."""
    with pytest.raises(ValueError, match="Unknown controller model"):
        make_controller_node("unknown", _ppp_config_path())
