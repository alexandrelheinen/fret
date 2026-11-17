"""FR-SYS-09: adding a model needs one profile YAML + one variant module.

This test proves the claim end to end using the *real*, unmodified
production facades (``fret.control.kinematics.Kinematics`` and
``fret.control.controller_node.make_controller_node``): a brand-new
"toy" 2-DOF arm, registered only here (never imported by
``fret.robots.variants``), resolves and runs through them with zero
edits to ``control/``, ``planning/``, ``ros/``, ``scene/``, ``vision/``
or ``launch/``.
"""

from __future__ import annotations

import pathlib

import numpy as np
import numpy.typing as npt
import pytest

from fret.robots.registry import (
    RobotRegistrationError,
    register_controller,
    register_kinematics,
    reset_default_registry,
)

_FIXTURES_DIR = pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "robots"


class _ToyKinematics:
    """Trivial 2-DOF planar kinematics satisfying ``KinematicsBackend``."""

    @property
    def dof(self) -> int:
        return 2

    @property
    def joint_names(self) -> list[str]:
        return ["toy_joint_1", "toy_joint_2"]

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        return np.array([[-3.14, 3.14], [-1.57, 1.57]], dtype=np.float64)

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        pose = np.eye(4, dtype=np.float64)
        q = np.asarray(joint_positions, dtype=np.float64)
        pose[0, 3] = float(np.sum(q))
        return pose

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        return np.zeros(2, dtype=np.float64)

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        return np.ones((6, 2), dtype=np.float64)


class _ToyController:
    """Minimal controller stand-in; proves the factory wiring, not tracking."""

    def __init__(self, model: str, config_path: str) -> None:
        self.model = model
        self.config_path = config_path


def _toy_controller_factory(model: str, config_path: str) -> _ToyController:
    return _ToyController(model=model, config_path=config_path)


# Register once at import time, like a real ``robots/variants/*.py`` module
# would. Guarded so re-importing this test module within one session does
# not raise "already registered" (module-level code only runs once).
try:
    register_kinematics("toy_arm")(_ToyKinematics)
    register_controller("toy_arm")(_toy_controller_factory)
except RobotRegistrationError:  # pragma: no cover
    pass


@pytest.fixture
def _toy_default_registry(monkeypatch: pytest.MonkeyPatch) -> None:
    """Point the process-wide default registry at the toy fixtures dir only."""
    import fret.robots.registry as registry_module

    monkeypatch.setattr(
        registry_module, "_default_profiles_dir", lambda: _FIXTURES_DIR
    )
    reset_default_registry()
    yield
    reset_default_registry()


class TestToyModelEndToEnd:
    def test_kinematics_facade_resolves_toy_model_unmodified(
        self, _toy_default_registry: None
    ) -> None:
        """``fret.control.kinematics.Kinematics`` needs no edits (FR-SYS-09)."""
        from fret.control.kinematics import Kinematics

        kin = Kinematics("toy")
        assert kin.dof == 2
        assert kin.joint_names == ["toy_joint_1", "toy_joint_2"]

    def test_make_controller_node_builds_toy_controller_unmodified(
        self, _toy_default_registry: None
    ) -> None:
        """``make_controller_node`` needs no edits (FR-SYS-09)."""
        from fret.control.controller_node import make_controller_node

        controller = make_controller_node("toy_arm", str(_FIXTURES_DIR))
        assert isinstance(controller, _ToyController)
        assert controller.model == "toy_arm"

    def test_unknown_alias_for_toy_model_still_rejected(
        self, _toy_default_registry: None
    ) -> None:
        from fret.robots.registry import UnknownRobotModelError, get_default_registry

        registry = get_default_registry()
        assert registry.resolve_key("toy") == "toy_arm"
        with pytest.raises(UnknownRobotModelError):
            registry.resolve_key("open_manipulator_x")
