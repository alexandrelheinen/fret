"""Deterministic ARC planner RNG for reproducible physics SITL runs.

ARCO planners call ``np.random.default_rng()`` without a seed.  Unseeded
plans are often untrackable under MuJoCo physics, which made release
showcase export flaky (~40% failure on CI) while still producing pose logs
that could be time-compressed into misleading clips.
"""

from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager

import numpy as np

# Matches tests/integration/conftest.py and test_mujoco_physics_dubins.py.
SHOWCASE_PLANNER_RNG_SEED: int = 11


@contextmanager
def deterministic_planner_rng(
    seed: int = SHOWCASE_PLANNER_RNG_SEED,
) -> Generator[None, None, None]:
    """Pin ``np.random.default_rng()`` for the duration of planner sampling."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(
        requested_seed: int | None = None,
    ) -> np.random.Generator:
        return original_default_rng(
            seed if requested_seed is None else requested_seed
        )

    np.random.default_rng = _seeded_default_rng  # type: ignore[assignment]
    try:
        yield
    finally:
        np.random.default_rng = original_default_rng
