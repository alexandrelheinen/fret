"""Deterministic ARC planner RNG (no scenario package import chain)."""

from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager

import numpy as np

SHOWCASE_PLANNER_RNG_SEED: int = 5

_active_seed: int | None = None


def active_planner_seed() -> int | None:
    """Return the seed of the innermost active ``deterministic_planner_rng``.

    ARCO v0.5.0 runs its planners as compiled code with their own random
    number generator, so patching ``numpy.random.default_rng`` no longer
    reaches them. Construction sites pass this value through the planners'
    ``seed`` argument instead; ``None`` means nothing pinned the run and
    the planner seeds itself from the system entropy, as before.
    """
    return _active_seed


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

    global _active_seed
    previous_seed = _active_seed
    _active_seed = int(seed)
    np.random.default_rng = _seeded_default_rng  # type: ignore[assignment]
    try:
        yield
    finally:
        np.random.default_rng = original_default_rng
        _active_seed = previous_seed
