"""Tests for fret.control.ControllerNode.

Acceptance criteria (FR-CTL-01 through FR-CTL-06):
  - Construction with model and config_path succeeds.
  - ``/controller_fault`` publishes True when EE position error > 20 mm.
  - In HALTED state, joint commands are all zeroed.
  - The controller timer fires at 50 Hz (timing tolerance ± 5 ms).
  - State transitions: IDLE → TRACKING on trajectory receipt,
    TRACKING → HALTED on fault.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.controller_node import ControllerNode


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction(tmp_path: pytest.TempPathFactory) -> None:
    ControllerNode(model="scara", config_path=str(tmp_path))


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_fault_published_on_large_error(tmp_path: pytest.TempPathFactory) -> None:
    """A tracking error > 20 mm must trigger a fault (FR-CTL-04)."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    fault = ctrl._check_fault(error_m=0.025)  # type: ignore[attr-defined]
    assert fault is True


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_no_fault_on_small_error(tmp_path: pytest.TempPathFactory) -> None:
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    fault = ctrl._check_fault(error_m=0.003)  # type: ignore[attr-defined]
    assert fault is False


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_halted_state_zeroes_commands(tmp_path: pytest.TempPathFactory) -> None:
    """In HALTED state every joint command must be zero (FR-CTL-05)."""
    ctrl = ControllerNode(model="scara", config_path=str(tmp_path))
    ctrl._enter_halted()  # type: ignore[attr-defined]
    cmd = ctrl._get_current_command()  # type: ignore[attr-defined]
    np.testing.assert_array_equal(cmd, np.zeros_like(cmd))
