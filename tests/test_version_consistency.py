"""FR-SYS-14: package.xml and pyproject.toml versions must be equal."""

from __future__ import annotations

import pathlib
import re
import tomllib

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


def _pyproject_version() -> str:
    data = tomllib.loads(
        (_REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    )
    version = data["project"]["version"]
    assert isinstance(version, str)
    return version


def _package_xml_version() -> str:
    text = (_REPO_ROOT / "src" / "fret" / "package.xml").read_text(
        encoding="utf-8"
    )
    match = re.search(r"<version>([^<]+)</version>", text)
    assert match is not None, "package.xml has no <version> element"
    return match.group(1)


def test_package_xml_and_pyproject_versions_are_equal() -> None:
    pyproject_version = _pyproject_version()
    package_xml_version = _package_xml_version()
    assert package_xml_version == pyproject_version, (
        f"src/fret/package.xml version ({package_xml_version!r}) and "
        f"pyproject.toml version ({pyproject_version!r}) must match "
        "(FR-SYS-14)"
    )
