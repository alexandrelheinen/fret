"""Tests for fret.control.controller_node (Level 3 logic + registry wiring).

Acceptance criteria covered:
    - FR-SYS-08: ``make_controller_node`` builds a controller for every
      registered arm model through the robot registry (T11), including
      OMY, which ``ControllerNode`` previously could not serve correctly
      (hardcoded DOF=4 and a hardcoded ``open_manipulator_x.yml`` file
      name when given a directory).
    - FR-CTL-01/04/06: basic FSM/tracking/fault behavior of the Level-3
      logic core.
"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from fret.control.controller_node import ControllerNode, make_controller_node

_CONTROLLERS_DIR = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "controllers"
)


class TestMakeControllerNodeThroughRegistry:
    @pytest.mark.parametrize(
        ("model", "expected_dof"),
        [
            ("open_manipulator_x", 4),
            ("omx", 4),
            ("open_manipulator_y", 6),
            ("omy", 6),
            ("six_dof", 6),
        ],
    )
    def test_builds_controller_with_correct_dof_from_directory(
        self, model: str, expected_dof: int
    ) -> None:
        controller = make_controller_node(model, str(_CONTROLLERS_DIR))
        assert isinstance(controller, ControllerNode)
        assert controller._dof == expected_dof

    def test_unknown_model_raises(self) -> None:
        from fret.robots.registry import UnknownRobotModelError

        with pytest.raises(UnknownRobotModelError):
            make_controller_node("not_a_model", str(_CONTROLLERS_DIR))

    def test_mobile_model_has_no_controller_factory_registered(self) -> None:
        """Dubins has no Jacobian ControllerNode variant registered."""
        from fret.robots.registry import RobotVariantNotRegisteredError

        with pytest.raises(RobotVariantNotRegisteredError):
            make_controller_node("dubins", str(_CONTROLLERS_DIR))

    def test_omy_loads_its_own_controller_config_not_omx(self) -> None:
        """Regression: directory-mode config loading previously always
        read ``open_manipulator_x.yml`` regardless of model."""
        controller = make_controller_node("omy", str(_CONTROLLERS_DIR))
        # open_manipulator_y.yml sets ticks_per_waypoint: 5 and
        # max_joint_velocity: 1.2, distinct from OM-X's 1.57.
        assert controller._max_joint_velocity == pytest.approx(1.2)


class TestControllerNodeTrajectoryTracking:
    def test_set_trajectory_requires_two_waypoints(self) -> None:
        controller = ControllerNode(
            model="open_manipulator_x",
            config_path=str(_CONTROLLERS_DIR),
            dof=4,
        )
        with pytest.raises(ValueError):
            controller.set_trajectory([np.zeros(4)])

    def test_current_waypoint_advances_with_joint_tracking(self) -> None:
        controller = ControllerNode(
            model="open_manipulator_x",
            config_path=str(_CONTROLLERS_DIR),
            dof=4,
        )
        wp0 = np.zeros(4, dtype=np.float64)
        wp1 = np.full(4, 0.5, dtype=np.float64)
        controller.set_trajectory([wp0, wp1])
        assert controller.has_trajectory()
        assert not controller.is_trajectory_complete()
        np.testing.assert_allclose(controller.current_waypoint(), wp0)

        # Close enough to wp0 -> advances to wp1.
        controller.compute_joint_command(wp0, joint_tol_rad=0.10)
        np.testing.assert_allclose(controller.current_waypoint(), wp1)
