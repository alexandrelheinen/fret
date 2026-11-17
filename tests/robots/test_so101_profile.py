"""SO-101 profile + registry wiring (FR-SIM-15, FR-SYS-06, FR-SYS-08)."""

from __future__ import annotations

import numpy as np

from fret.control.kinematics import Kinematics
from fret.robots.registry import RobotRegistry, reset_default_registry

_ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def setup_function() -> None:
    reset_default_registry()


def teardown_function() -> None:
    reset_default_registry()


def test_so101_profile_resolves_through_default_registry() -> None:
    registry = RobotRegistry()
    profile = registry.get_profile("so101")
    assert profile.kind == "arm"
    assert list(profile.joint_names) == _ARM_JOINTS
    assert profile.dof == 5
    assert profile.grasp_modes == ("friction",)


def test_so101_aliases_resolve_to_canonical_key() -> None:
    registry = RobotRegistry()
    assert registry.resolve_key("so-101") == "so101"
    assert registry.resolve_key("so_arm101") == "so101"


def test_kinematics_facade_works_through_the_registry() -> None:
    """T7 done-when: Kinematics("so101") works through the registry."""
    kin = Kinematics("so101")
    assert kin.dof == 5
    assert kin.joint_names == _ARM_JOINTS
    q_mid = kin.joint_limits.mean(axis=1)
    T = kin.forward_kinematics(q_mid)
    assert T.shape == (4, 4)
    assert np.isfinite(T).all()


def test_so101_controller_factory_registered() -> None:
    registry = RobotRegistry()
    factory = registry.get_controller_factory("so101")
    assert callable(factory)
