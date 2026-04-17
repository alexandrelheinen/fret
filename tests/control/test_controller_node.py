"""Tests for fret.control.ControllerNode — Level 3 logic core.

Acceptance criteria (FR-CTL-01 through FR-CTL-06):
  - Construction with model and config_path succeeds.
  - ``_check_fault(error_m)`` returns True when error > 20 mm (FR-CTL-04).
  - ``_check_fault(error_m)`` returns False when error ≤ 20 mm.
  - ``_enter_halted()`` transitions state to HALTED and zeroes commands (FR-CTL-05).
  - ``_get_current_command()`` returns a zero array on construction.
  - Config loading from YAML applies custom threshold values.
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest
import yaml

from fret.control.controller_node import ControllerNode


def test_construction(tmp_path: pathlib.Path) -> None:
    """ControllerNode can be constructed with model and config_path."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    assert ctrl is not None


def test_initial_command_is_zero(tmp_path: pathlib.Path) -> None:
    """Command vector must be zero on construction."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    cmd = ctrl._get_current_command()
    np.testing.assert_array_equal(cmd, np.zeros_like(cmd))


def test_initial_command_shape(tmp_path: pathlib.Path) -> None:
    """Command vector must have shape (DOF,) = (3,) for SCARA."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    cmd = ctrl._get_current_command()
    assert cmd.shape == (3,)


def test_fault_published_on_large_error(tmp_path: pathlib.Path) -> None:
    """A tracking error > 20 mm must trigger a fault (FR-CTL-04)."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    assert ctrl._check_fault(error_m=0.025) is True


def test_fault_on_boundary_value(tmp_path: pathlib.Path) -> None:
    """Error exactly at threshold (0.020 m) must NOT trigger a fault."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    assert ctrl._check_fault(error_m=0.020) is False


def test_no_fault_on_small_error(tmp_path: pathlib.Path) -> None:
    """Error below the threshold must not trigger a fault."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    assert ctrl._check_fault(error_m=0.003) is False


def test_halted_state_zeroes_commands(tmp_path: pathlib.Path) -> None:
    """In HALTED state every joint command must be zero (FR-CTL-05)."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    ctrl._enter_halted()
    cmd = ctrl._get_current_command()
    np.testing.assert_array_equal(cmd, np.zeros_like(cmd))


def test_get_current_command_returns_copy(tmp_path: pathlib.Path) -> None:
    """Modifying the returned command must not affect internal state."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    cmd = ctrl._get_current_command()
    cmd[0] = 99.0
    cmd2 = ctrl._get_current_command()
    assert cmd2[0] == 0.0


def test_config_loading_from_yaml(tmp_path: pathlib.Path) -> None:
    """Custom threshold from a YAML file must override the default."""
    config_file = tmp_path / "jacobian.yml"
    config_data = {
        "/**": {
            "ros__parameters": {
                "fault_threshold": 0.050,
                "damping_factor": 0.005,
                "max_joint_velocity": 2.0,
                "update_rate": 100.0,
            }
        }
    }
    config_file.write_text(yaml.dump(config_data))
    ctrl = ControllerNode(model="scara", config_path=str(config_file))
    # 0.040 m is below the new threshold (0.050), so no fault
    assert ctrl._check_fault(error_m=0.040) is False
    # 0.060 m exceeds the new threshold
    assert ctrl._check_fault(error_m=0.060) is True


def test_config_missing_file_uses_defaults(tmp_path: pathlib.Path) -> None:
    """If the YAML file does not exist, defaults must be used silently."""
    ctrl = ControllerNode(
        model="scara", config_path=str(tmp_path / "nonexistent.yml")
    )
    # Default threshold is 0.020 m
    assert ctrl._check_fault(error_m=0.021) is True
    assert ctrl._check_fault(error_m=0.019) is False
