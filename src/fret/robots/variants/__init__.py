"""Per-model-family variant registrations (FR-SYS-08, FR-SYS-09).

Importing this package has the side effect of registering every built-in
model's kinematics (and, where applicable, controller) factories with
:mod:`fret.robots.registry`. Adding a new model means adding one module
here (or in a test fixture package, for test-only models) and one profile
YAML under ``config/robots/`` — no edits to ``control/``, ``planning/``,
``ros/``, ``scene/``, ``vision/`` or ``launch/`` (FR-SYS-09).
"""

from __future__ import annotations

from fret.robots.variants import (  # noqa: F401
    dubins,
    open_manipulator_x,
    open_manipulator_y,
    so101,
)

__all__ = ["dubins", "open_manipulator_x", "open_manipulator_y", "so101"]
