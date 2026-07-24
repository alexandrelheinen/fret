"""Simulation test package setup.

MuJoCo picks its GL backend on first import. Force EGL for headless CI before
any test module imports ``mujoco``.
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
