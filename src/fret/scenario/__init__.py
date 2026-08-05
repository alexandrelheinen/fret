"""Scenario orchestration for FRET release showcases.

- v1.1: ``DubinsRaceRunner`` (SC-v11)

Heavy Dubins / MuJoCo / vision imports are lazy so lightweight helpers
(``planner_rng``) used by arm pick-place planning do not pull OpenCV.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

__all__ = [
    "DubinsRaceRunResult",
    "DubinsRaceRunner",
    "DubinsRaceSimulation",
]

if TYPE_CHECKING:
    from fret.scenario.dubins_race_runner import (
        DubinsRaceRunner,
        DubinsRaceRunResult,
        DubinsRaceSimulation,
    )


def __getattr__(name: str) -> Any:
    if name in {
        "DubinsRaceRunner",
        "DubinsRaceRunResult",
        "DubinsRaceSimulation",
    }:
        from fret.scenario import dubins_race_runner as _runner

        return getattr(_runner, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
