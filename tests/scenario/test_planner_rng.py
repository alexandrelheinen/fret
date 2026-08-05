"""Tests for deterministic ARC planner RNG pinning."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

from fret.scenario.planner_rng import (
    SHOWCASE_PLANNER_RNG_SEED,
    deterministic_planner_rng,
)


def test_deterministic_planner_rng_pins_default_rng() -> None:
    expected = np.random.default_rng(SHOWCASE_PLANNER_RNG_SEED).integers(
        0,
        1_000_000,
        3,
    )
    with deterministic_planner_rng(SHOWCASE_PLANNER_RNG_SEED):
        actual = np.random.default_rng().integers(0, 1_000_000, 3)
    assert list(actual) == list(expected)


def test_deterministic_planner_rng_honours_a_non_default_seed() -> None:
    """Regression test: a custom ``seed`` must not fall back to the default.

    An earlier implementation named the wrapped call's own seed argument
    ``seed`` too, which shadowed the outer ``seed`` this context manager
    was configured with — every call silently pinned to
    ``SHOWCASE_PLANNER_RNG_SEED`` no matter what seed was requested.
    """
    custom_seed = 42
    expected = np.random.default_rng(custom_seed).integers(0, 1_000_000, 3)
    with deterministic_planner_rng(custom_seed):
        actual = np.random.default_rng().integers(0, 1_000_000, 3)
    assert list(actual) == list(expected)

    default_expected = np.random.default_rng(
        SHOWCASE_PLANNER_RNG_SEED
    ).integers(0, 1_000_000, 3)
    assert list(actual) != list(default_expected)


def test_planner_rng_import_does_not_load_opencv() -> None:
    """Arm planners import planner_rng; that must not pull broken OpenCV."""
    src = Path(__file__).resolve().parents[2] / "src"
    proc = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; "
            f"sys.path.insert(0, {str(src)!r}); "
            "from fret.scenario.planner_rng import deterministic_planner_rng; "
            "assert 'cv2' not in sys.modules; "
            "assert 'fret.scenario.dubins_race_runner' not in sys.modules; "
            "print('ok')",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert "ok" in proc.stdout
