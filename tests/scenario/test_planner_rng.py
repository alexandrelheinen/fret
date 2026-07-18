"""Tests for deterministic ARC planner RNG pinning."""

from __future__ import annotations

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
