"""Tests for fret.robots.registry (FR-SYS-06, FR-SYS-08)."""

from __future__ import annotations

import pathlib

import pytest
import yaml

from fret.robots.registry import (
    RobotRegistrationError,
    RobotRegistry,
    RobotVariantNotRegisteredError,
    UnknownRobotModelError,
    get_default_registry,
)

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_REAL_PROFILES_DIR = _REPO_ROOT / "src" / "fret" / "config" / "robots"


def _write_yaml(path: pathlib.Path, data: dict) -> None:
    path.write_text(yaml.safe_dump(data), encoding="utf-8")


_BASE: dict = {
    "name": "Test",
    "kind": "arm",
    "joint_names": ["j1"],
    "joint_limits": [[-1.0, 1.0]],
    "kinematics": "test_kin",
    "controller_config": "config/controllers/test.yml",
    "planning_config": "planning/test.yml",
}


class TestDefaultRegistryBuiltins:
    """The real config/robots/ profiles resolve by canonical key and alias."""

    def test_resolves_canonical_key_and_aliases(self) -> None:
        registry = get_default_registry()
        for key, aliases in (
            ("dubins", ()),
            ("open_manipulator_x", ("omx",)),
            ("open_manipulator_y", ("omy", "six_dof")),
        ):
            assert registry.resolve_key(key) == key
            for alias in aliases:
                assert registry.resolve_key(alias) == key

    def test_unknown_key_raises_with_known_keys(self) -> None:
        registry = get_default_registry()
        with pytest.raises(UnknownRobotModelError) as exc_info:
            registry.resolve_key("not_a_model")
        message = str(exc_info.value)
        assert "not_a_model" in message
        for key in ("dubins", "open_manipulator_x", "omx", "omy"):
            assert key in message
        assert exc_info.value.known_keys == registry.known_keys()

    @pytest.mark.parametrize(
        "variant",
        [" open_manipulator_x", "open_manipulator_x ", "OMX", "Dubins"],
    )
    def test_key_with_case_or_whitespace_variation_is_rejected(
        self, variant: str
    ) -> None:
        """Edge case: keys are matched exactly, never case-/space-folded."""
        registry = get_default_registry()
        with pytest.raises(UnknownRobotModelError):
            registry.resolve_key(variant)

    def test_get_kinematics_returns_working_backend(self) -> None:
        registry = get_default_registry()
        kin = registry.get_kinematics("omx")
        assert kin.dof == 4
        kin_omy = registry.get_kinematics("six_dof")
        assert kin_omy.dof == 6


class TestRegistryIsolatedConstruction:
    """A fresh RobotRegistry() is rebuilt from files, never shared state."""

    def test_two_registries_from_same_files_agree(self) -> None:
        a = RobotRegistry(_REAL_PROFILES_DIR)
        b = RobotRegistry(_REAL_PROFILES_DIR)
        assert a.known_keys() == b.known_keys()
        assert a.get_profile("dubins") == b.get_profile("dubins")

    def test_profiles_dir_not_found_raises(
        self, tmp_path: pathlib.Path
    ) -> None:
        with pytest.raises(FileNotFoundError):
            RobotRegistry(tmp_path / "does_not_exist")


class TestAliasCollision:
    """Edge case: a colliding alias fails to load, naming both files."""

    def test_alias_collides_with_another_models_alias(
        self, tmp_path: pathlib.Path
    ) -> None:
        a = dict(_BASE, aliases=["dup"])
        b = dict(_BASE, aliases=["dup"])
        _write_yaml(tmp_path / "model_a.yml", a)
        _write_yaml(tmp_path / "model_b.yml", b)
        with pytest.raises(RobotRegistrationError) as exc_info:
            RobotRegistry(tmp_path)
        message = str(exc_info.value)
        assert "model_a.yml" in message
        assert "model_b.yml" in message

    def test_alias_collides_with_another_models_canonical_key(
        self, tmp_path: pathlib.Path
    ) -> None:
        a = dict(_BASE)
        b = dict(_BASE, aliases=["model_a"])
        _write_yaml(tmp_path / "model_a.yml", a)
        _write_yaml(tmp_path / "model_b.yml", b)
        with pytest.raises(RobotRegistrationError) as exc_info:
            RobotRegistry(tmp_path)
        message = str(exc_info.value)
        assert "model_a" in message
        assert "model_b.yml" in message


class TestVariantNotRegistered:
    def test_missing_kinematics_factory_raises(
        self, tmp_path: pathlib.Path
    ) -> None:
        _write_yaml(
            tmp_path / "ghost.yml", dict(_BASE, kinematics="ghost_kin")
        )
        registry = RobotRegistry(
            tmp_path, kinematics_factories={}, controller_factories={}
        )
        with pytest.raises(RobotVariantNotRegisteredError):
            registry.get_kinematics("ghost")
