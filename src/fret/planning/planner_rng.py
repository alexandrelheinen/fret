"""Deterministic ARC planner RNG (no scenario package import chain)."""

from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager

import numpy as np

SHOWCASE_PLANNER_RNG_SEED: int = 5


@contextmanager
def deterministic_planner_rng(
    seed: int = SHOWCASE_PLANNER_RNG_SEED,
) -> Generator[None, None, None]:
    """Pin ``np.random.default_rng()`` for the duration of planner sampling."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(
        call_seed: int | None = None,
        **kwargs: object,
    ) -> np.random.Generator:
        pinned = call_seed if call_seed is not None else seed
        return original_default_rng(pinned, **kwargs)

    np.random.default_rng = _seeded_default_rng  # type: ignore[assignment]
    try:
        yield
    finally:
        np.random.default_rng = original_default_rng
