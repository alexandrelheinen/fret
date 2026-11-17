"""FR-SYS-13: docs outside docs/robots/ and docs/hardware/ stay hardware-neutral.

Grep over every Markdown file under ``docs/`` (and the top-level
``README.md``), excluding ``docs/robots/`` and ``docs/hardware/``, for a
specific hardware target named as a software assumption: Raspberry Pi,
Arduino, Nema(-class actuator), or Micro-ROS. Word-boundary matched so
"kinematics" (contains "nema") is not a false positive.
"""

from __future__ import annotations

import pathlib
import re

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
_DOCS_ROOT = _REPO_ROOT / "docs"
_EXEMPT_DIR_NAMES = frozenset({"robots", "hardware"})

_HARDWARE_PATTERN = re.compile(
    r"raspberry\s*pi(\s*5)?|arduino(\s*mega)?|\bnema\b|micro-?ros",
    re.IGNORECASE,
)


def _iter_docs_files() -> list[pathlib.Path]:
    files = [_REPO_ROOT / "README.md"]
    for path in sorted(_DOCS_ROOT.rglob("*.md")):
        rel = path.relative_to(_DOCS_ROOT)
        if rel.parts and rel.parts[0] in _EXEMPT_DIR_NAMES:
            continue
        files.append(path)
    return files


def _requirements_hardware_section_lines(text: str) -> set[int]:
    """Line numbers (1-based) inside requirements.md's "## Hardware" section.

    FR-SYS-13 keeps FR-HW-01..03's original wording verbatim, annotated
    "superseded", not renumbered or rewritten — so that one historical
    section is exempt, not the whole file.
    """
    lines = text.splitlines()
    exempt: set[int] = set()
    in_section = False
    for i, line in enumerate(lines, start=1):
        if line.strip().startswith("## Hardware"):
            in_section = True
            exempt.add(i)
            continue
        if in_section:
            if line.strip().startswith("## "):
                break
            exempt.add(i)
    return exempt


def _fr_sys_13_paragraph_lines(lines: list[str]) -> set[int]:
    """Line numbers (1-based) in the paragraph that mentions FR-SYS-13 itself.

    The requirement text necessarily names its own forbidden examples
    parenthetically ("shall not name ... (Raspberry Pi, Arduino, ...)");
    that is the rule's definition, not an assumption of the software.
    Markdown paragraphs are separated by blank lines.
    """
    exempt: set[int] = set()
    paragraph: list[int] = []
    has_marker = False
    for i, line in enumerate(lines, start=1):
        if line.strip() == "":
            if has_marker:
                exempt.update(paragraph)
            paragraph = []
            has_marker = False
            continue
        paragraph.append(i)
        if "FR-SYS-13" in line:
            has_marker = True
    if has_marker:
        exempt.update(paragraph)
    return exempt


def test_docs_outside_robots_and_hardware_name_no_hardware_target() -> None:
    violations: list[str] = []
    for path in _iter_docs_files():
        text = path.read_text(encoding="utf-8")
        lines = text.splitlines()
        exempt_lines = (
            _requirements_hardware_section_lines(text)
            if path.name == "requirements.md"
            else set()
        )
        exempt_lines |= _fr_sys_13_paragraph_lines(lines)
        for lineno, line in enumerate(lines, start=1):
            if lineno in exempt_lines:
                continue
            match = _HARDWARE_PATTERN.search(line)
            if match:
                violations.append(f"{path}:{lineno}: {match.group(0)!r}")
    assert not violations, (
        "Hardware-target mention(s) found outside docs/robots/, "
        "docs/hardware/ (FR-SYS-13):\n" + "\n".join(violations)
    )


def test_exempt_directories_exist() -> None:
    for name in sorted(_EXEMPT_DIR_NAMES):
        assert (_DOCS_ROOT / name).is_dir(), f"docs/{name}/ must exist"
