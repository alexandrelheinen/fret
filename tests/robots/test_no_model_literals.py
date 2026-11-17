"""FR-SYS-08 (AST scan): no model-name string-literal dispatch outside
``fret.robots`` / ``config/robots``.

Scans every ``.py`` file under ``src/fret`` (excluding ``src/fret/robots/``)
for a comparison (``==``, ``!=``, ``in``, ``not in``) between a variable and
a string literal that is a known robot model key or alias. A handful of
call sites have not yet been migrated to the registry as of PR1
(v1.5.0) — each is a named, explicit exception below, tracked as a
follow-up in ``docs/decisions.md``, not a silent allowance.

Prove the scanner actually fires by a temporary local edit (not committed):
add ``if model == "omx":`` to a non-exempt module under ``src/fret`` and
re-run this test; it must fail.
"""

from __future__ import annotations

import ast
import pathlib

_SRC_ROOT = pathlib.Path(__file__).resolve().parents[2] / "src" / "fret"

# Canonical keys + aliases declared by config/robots/*.yml (FR-SYS-06).
# Kept here, not imported from the registry, so the scan does not depend
# on a built registry (and stays meaningful for a model key that was
# *removed* from a profile but left behind in a stray comparison).
_MODEL_LITERALS: frozenset[str] = frozenset(
    {
        "dubins",
        "open_manipulator_x",
        "omx",
        "open_manipulator_y",
        "omy",
        "six_dof",
    }
)

# Files not yet migrated off model-literal dispatch as of PR1 (v1.5.0).
# Each is a real, tracked gap (docs/decisions.md: "PR1 robot-registry
# migration — remaining dispatch sites"), not a blanket exemption: every
# other file under src/fret (outside robots/) is held to FR-SYS-08 today.
_KNOWN_REMAINING_DISPATCH: frozenset[str] = frozenset(
    {
        "control/pick_place_planning.py",
        "control/pick_place_vision.py",
        "control/pick_place_clutter_sim.py",
        "ros/mujoco_bridge.py",
    }
)


def _iter_python_files() -> list[pathlib.Path]:
    files = []
    for path in sorted(_SRC_ROOT.rglob("*.py")):
        rel = path.relative_to(_SRC_ROOT)
        if rel.parts[0] == "robots":
            continue
        if "__pycache__" in rel.parts:
            continue
        if str(rel) in _KNOWN_REMAINING_DISPATCH:
            continue
        files.append(path)
    return files


def _string_literal(node: ast.expr) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _literal_collection_elements(node: ast.expr) -> list[ast.expr] | None:
    """Return element nodes if ``node`` is a set/list/tuple display, or a
    ``frozenset(...)``/``set(...)`` call over one — else ``None``."""
    if isinstance(node, (ast.Set, ast.List, ast.Tuple)):
        return list(node.elts)
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in {"frozenset", "set"}
        and len(node.args) == 1
        and isinstance(node.args[0], (ast.Set, ast.List, ast.Tuple))
    ):
        return list(node.args[0].elts)
    return None


def _names_bound_to_model_literal_collections(tree: ast.AST) -> set[str]:
    """Return variable names assigned a literal collection of model keys.

    Catches the ``_OMY_MODELS = frozenset({"omy", ...})`` pattern: the
    model literal lives in the collection, not directly in the later
    ``model in _OMY_MODELS`` comparison.
    """
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        elements = _literal_collection_elements(node.value)
        if elements is None:
            continue
        literals = {
            lit for el in elements if (lit := _string_literal(el)) is not None
        }
        if literals & _MODEL_LITERALS:
            names.add(target.id)
    return names


def _find_model_literal_comparisons(
    tree: ast.AST, path: pathlib.Path
) -> list[str]:
    hits: list[str] = []
    literal_collection_names = _names_bound_to_model_literal_collections(tree)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Compare):
            continue
        operands = [node.left, *node.comparators]
        literals = [
            lit
            for operand in operands
            if (lit := _string_literal(operand)) is not None
        ]
        for lit in literals:
            if lit in _MODEL_LITERALS:
                hits.append(
                    f"{path}:{node.lineno}: comparison against {lit!r}"
                )
        names = [
            operand.id for operand in operands if isinstance(operand, ast.Name)
        ]
        for name in names:
            if name in literal_collection_names:
                hits.append(
                    f"{path}:{node.lineno}: membership test against "
                    f"model-literal collection {name!r}"
                )
    return hits


def test_no_model_name_literal_comparisons_outside_robots_package() -> None:
    violations: list[str] = []
    for path in _iter_python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        violations.extend(_find_model_literal_comparisons(tree, path))
    assert not violations, (
        "Model-name string-literal comparison(s) found outside "
        "fret.robots / config/robots (FR-SYS-08):\n" + "\n".join(violations)
    )


def test_known_remaining_dispatch_list_still_exists_and_still_matches() -> (
    None
):
    """Guard against the exception list silently growing stale.

    Every exempted file must still exist and still contain at least one
    model-literal comparison; once a file is migrated, it must be removed
    from ``_KNOWN_REMAINING_DISPATCH`` in the same commit (no stale
    exceptions).
    """
    for rel in sorted(_KNOWN_REMAINING_DISPATCH):
        path = _SRC_ROOT / rel
        assert path.is_file(), f"exempted path no longer exists: {rel}"
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        hits = _find_model_literal_comparisons(tree, path)
        assert hits, (
            f"{rel} is exempted from FR-SYS-08 but no longer contains a "
            "model-literal comparison; remove it from "
            "_KNOWN_REMAINING_DISPATCH"
        )
