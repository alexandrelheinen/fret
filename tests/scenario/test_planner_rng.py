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
