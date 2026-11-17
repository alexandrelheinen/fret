"""Tests for fret.robots.profile (FR-SYS-07)."""

from __future__ import annotations

import pathlib

import pytest
import yaml

from fret.robots.profile import (
    RobotProfile,
    RobotProfileError,
    load_robot_profile,
)

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_REAL_PROFILES_DIR = _REPO_ROOT / "src" / "fret" / "config" / "robots"


def _write_profile(
    tmp_path: pathlib.Path, name: str, data: dict
) -> pathlib.Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


_VALID_ARM: dict = {
    "name": "Test Arm",
    "kind": "arm",
    "joint_names": ["j1", "j2"],
    "joint_limits": [[-1.0, 1.0], [-2.0, 2.0]],
    "kinematics": "test_arm",
    "controller_config": "config/controllers/test_arm.yml",
    "planning_config": "planning/test_arm.yml",
}


class TestRealProfiles:
    """Every shipped profile must load and validate."""

    @pytest.mark.parametrize(
        "stem", ["dubins", "open_manipulator_x", "open_manipulator_y"]
    )
    def test_real_profile_loads(self, stem: str) -> None:
        profile = load_robot_profile(_REAL_PROFILES_DIR / f"{stem}.yml")
        assert profile.key == stem
        assert profile.kind in {"mobile", "arm"}
        assert len(profile.joint_names) == len(profile.joint_limits)
        assert profile.dof == len(profile.joint_names)


class TestProfileValidation:
    def test_missing_file_raises_file_not_found(
        self, tmp_path: pathlib.Path
    ) -> None:
        with pytest.raises(FileNotFoundError):
            load_robot_profile(tmp_path / "missing.yml")

    def test_valid_profile_loads(self, tmp_path: pathlib.Path) -> None:
        path = _write_profile(tmp_path, "test_arm.yml", _VALID_ARM)
        profile = load_robot_profile(path)
        assert isinstance(profile, RobotProfile)
        assert profile.key == "test_arm"
        assert profile.name == "Test Arm"
        assert profile.kind == "arm"
        assert profile.joint_names == ("j1", "j2")
        assert profile.joint_limits == ((-1.0, 1.0), (-2.0, 2.0))
        assert profile.aliases == ()

    @pytest.mark.parametrize(
        "missing_field",
        [
            "name",
            "kind",
            "joint_names",
            "joint_limits",
            "kinematics",
            "controller_config",
            "planning_config",
        ],
    )
    def test_missing_required_field_names_field_and_file(
        self, tmp_path: pathlib.Path, missing_field: str
    ) -> None:
        data = dict(_VALID_ARM)
        del data[missing_field]
        path = _write_profile(tmp_path, "bad.yml", data)
        with pytest.raises(RobotProfileError) as exc_info:
            load_robot_profile(path)
        message = str(exc_info.value)
        assert missing_field in message
        assert "bad.yml" in message

    def test_invalid_kind_rejected(self, tmp_path: pathlib.Path) -> None:
        data = dict(_VALID_ARM)
        data["kind"] = "humanoid"
        path = _write_profile(tmp_path, "bad.yml", data)
        with pytest.raises(RobotProfileError, match="kind"):
            load_robot_profile(path)

    def test_joint_limits_length_mismatch_rejected(
        self, tmp_path: pathlib.Path
    ) -> None:
        data = dict(_VALID_ARM)
        data["joint_limits"] = [[-1.0, 1.0]]  # only 1, but 2 joint_names
        path = _write_profile(tmp_path, "bad.yml", data)
        with pytest.raises(RobotProfileError, match="joint_limits"):
            load_robot_profile(path)

    def test_mobile_kind_rejects_grasp_modes(
        self, tmp_path: pathlib.Path
    ) -> None:
        """Edge case: a mobile profile asking for a grasp mode is a typed
        error, not a silent default."""
        data = dict(_VALID_ARM)
        data["kind"] = "mobile"
        data["grasp_modes"] = ["parallel"]
        path = _write_profile(tmp_path, "bad.yml", data)
        with pytest.raises(RobotProfileError, match="grasp_modes"):
            load_robot_profile(path)

    def test_non_mapping_root_rejected(self, tmp_path: pathlib.Path) -> None:
        path = tmp_path / "bad.yml"
        path.write_text("- just\n- a\n- list\n", encoding="utf-8")
        with pytest.raises(RobotProfileError):
            load_robot_profile(path)

    def test_ill_typed_field_names_field_and_file(
        self, tmp_path: pathlib.Path
    ) -> None:
        data = dict(_VALID_ARM)
        data["name"] = 123  # must be a string
        path = _write_profile(tmp_path, "bad.yml", data)
        with pytest.raises(RobotProfileError) as exc_info:
            load_robot_profile(path)
        message = str(exc_info.value)
        assert "name" in message
        assert "bad.yml" in message

    def test_aliases_and_optional_fields_round_trip(
        self, tmp_path: pathlib.Path
    ) -> None:
        data = dict(_VALID_ARM)
        data["aliases"] = ["ta", "toy"]
        data["mjcf"] = "mjcf/test_arm.xml"
        data["vision_config"] = "vision/test_arm.yml"
        data["transfer_planner"] = "test_arm"
        data["uses_vision"] = True
        path = _write_profile(tmp_path, "test_arm.yml", data)
        profile = load_robot_profile(path)
        assert profile.aliases == ("ta", "toy")
        assert profile.mjcf == "mjcf/test_arm.xml"
        assert profile.vision_config == "vision/test_arm.yml"
        assert profile.transfer_planner == "test_arm"
        assert profile.uses_vision is True
