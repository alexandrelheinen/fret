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

# Matches integration gates; seed 5 gives smooth physics showcase paths at
# clearance_margin 1.0 + physics planning bump (v1.1.0-demo margins).
SHOWCASE_PLANNER_RNG_SEED: int = 5


@contextmanager
def deterministic_planner_rng(
    seed: int = SHOWCASE_PLANNER_RNG_SEED,
) -> Generator[None, None, None]:
    """Pin ``np.random.default_rng()`` for the duration of planner sampling."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(
        seed: int | None = None,
        **kwargs: object,
    ) -> np.random.Generator:
        """Match ``numpy.random.default_rng`` signature for patched calls."""
        pinned = seed if seed is not None else SHOWCASE_PLANNER_RNG_SEED
        return original_default_rng(pinned, **kwargs)

    np.random.default_rng = _seeded_default_rng  # type: ignore[assignment]
    try:
        yield
    finally:
        np.random.default_rng = original_default_rng
