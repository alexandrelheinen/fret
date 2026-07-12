"""Non-holonomic wheel constraints for Dubins physics SITL (v1.2).

Holonomic X/Y slide joints are a deliberate MJCF simplification
(see ``docs/mujoco_physics_v1.2.md``).  After each ``mj_step`` we project
planar ``qvel`` so motion is forward-only in the body frame — matching
what differential-drive wheels can produce.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import numpy.typing as npt


def project_planar_velocity_forward_only(
    vx: float,
    vy: float,
    heading_rad: float,
) -> tuple[float, float]:
    """Zero lateral body velocity; keep forward component along ``heading_rad``."""
    forward = vx * math.cos(heading_rad) + vy * math.sin(heading_rad)
    return (
        forward * math.cos(heading_rad),
        forward * math.sin(heading_rad),
    )


def enforce_slide_yaw_nonholonomic_qvel(
    data: Any,
    qpos_adrs: dict[str, int],
    qvel_adrs: dict[str, int],
    joint_names: tuple[str, str, str],
) -> None:
    """Project one agent's slide-X/Y ``qvel`` to forward-only body motion."""
    jx, jy, jyaw = joint_names
    heading = float(data.qpos[qpos_adrs[jyaw]])
    vx, vy = project_planar_velocity_forward_only(
        float(data.qvel[qvel_adrs[jx]]),
        float(data.qvel[qvel_adrs[jy]]),
        heading,
    )
    data.qvel[qvel_adrs[jx]] = vx
    data.qvel[qvel_adrs[jy]] = vy


def max_body_lateral_speed_m_s(
    pose_history: (
        npt.NDArray[np.float64] | tuple[tuple[float, float, float], ...]
    ),
    *,
    dt: float,
    max_yaw_rate_rad_s: float | None = None,
) -> float:
    """Return peak |lateral speed| inferred from a planar pose log [m/s].

    When ``max_yaw_rate_rad_s`` is set, samples during sharp turns are skipped
    so cornering curvature is not mistaken for holonomic skid.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    poses = np.asarray(pose_history, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[0] < 2 or poses.shape[1] < 3:
        return 0.0

    peak = 0.0
    for idx in range(poses.shape[0] - 1):
        yaw_rate = abs(float(poses[idx + 1, 2] - poses[idx, 2])) / dt
        if max_yaw_rate_rad_s is not None and yaw_rate > max_yaw_rate_rad_s:
            continue
        x0, y0, theta = poses[idx, :3]
        dx = float(poses[idx + 1, 0] - x0)
        dy = float(poses[idx + 1, 1] - y0)
        sin_t = math.sin(float(theta))
        cos_t = math.cos(float(theta))
        lateral = (-dx * sin_t + dy * cos_t) / dt
        peak = max(peak, abs(lateral))
    return peak
