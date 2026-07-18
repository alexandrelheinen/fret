"""Scene acquisition layer.

Re-exports the public API of the scene sub-package.
"""

from fret.scene.acquisition import SceneAcquisition
from fret.scene.occupancy_adapter import OccupancyAdapter

__all__ = [
    "OccupancyAdapter",
    "SceneAcquisition",
]
