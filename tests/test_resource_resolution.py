"""FR-SYS-10: package resources resolve from any working directory.

Runs with a ``tmp_path`` as the current working directory to prove that
``fret.resources.resolve`` (and the call sites built on it:
``config_loader``, ``sitl_config``) never depends on ``Path.cwd()``.
"""

from __future__ import annotations

import pathlib

import pytest

from fret.resources import package_share_root, resolve


class TestResolveIndependentOfCwd:
    def test_config_file_resolves_with_unrelated_cwd(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        path = resolve("config", "robots/dubins.yml")
        assert path.is_file()
        assert path.name == "dubins.yml"

    def test_mjcf_dir_root_resolves_with_unrelated_cwd(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        path = resolve("config", "scenarios/omx_reach.yml")
        assert path.is_file()

    def test_missing_resource_raises_naming_the_path(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        with pytest.raises(FileNotFoundError, match="does_not_exist.yml"):
            resolve("config", "robots/does_not_exist.yml")

    def test_package_share_root_is_absolute_and_cwd_independent(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        root = package_share_root()
        assert root.is_absolute()
        assert root != tmp_path


class TestConfigLoaderCallSitesIndependentOfCwd:
    """Downstream call sites inherit the CWD-independence (FR-SYS-10)."""

    def test_planning_config_for_model_from_unrelated_cwd(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        from fret.config_loader import (
            load_algorithm_config,
            planning_config_for_model,
        )

        rel = planning_config_for_model("open_manipulator_x")
        config = load_algorithm_config(rel)
        assert isinstance(config, dict)

    def test_controller_config_relative_from_unrelated_cwd(
        self, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        from fret.sitl_config import (
            controller_config_relative,
            resolve_package_file,
        )

        rel = controller_config_relative("omy")
        path = resolve_package_file(*rel.split("/"))
        assert path.is_file()
