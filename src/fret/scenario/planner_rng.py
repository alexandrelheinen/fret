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
        """Match ``numpy.random.default_rng`` signature for patched calls.

        ``call_seed`` is whatever the wrapped call site passes (ARCO's
        planners call ``np.random.default_rng()`` with no argument, so this
        is normally ``None``). It must not be named ``seed`` — that would
        shadow the outer ``seed`` this context manager was configured with,
        silently pinning every call to ``SHOWCASE_PLANNER_RNG_SEED``
        regardless of what the caller asked for.
        """
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
