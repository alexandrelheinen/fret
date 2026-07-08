"""PPP gantry collision envelopes derived from MJCF geometry (T10-05).

Maps configuration ``q = (x, y, z)`` (EE / ``z_hoist`` origin in world
frame) to axis-aligned bounding boxes for all moving gantry bodies.

Satisfies requirement FR-PLN-02.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class BodyEnvelope:
    """Axis-aligned box in world frame."""

    centre: npt.NDArray[np.float64]
    half_extent: npt.NDArray[np.float64]


def _box_corners(
    centre: npt.NDArray[np.float64],
    half_extent: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    hx, hy, hz = half_extent
    offsets = np.array(
        [
            [-hx, -hy, -hz],
            [hx, -hy, -hz],
            [-hx, hy, -hz],
            [hx, hy, -hz],
            [-hx, -hy, hz],
            [hx, -hy, hz],
            [-hx, hy, hz],
            [hx, hy, hz],
        ],
        dtype=np.float64,
    )
    return centre + offsets


def ppp_body_envelopes(
    configuration: npt.NDArray[np.float64],
) -> list[BodyEnvelope]:
    """Return world-frame AABBs for moving PPP gantry bodies.

    Geometry matches ``mjcf/ppp_warehouse.xml`` at 1:5 preview scale.
    Fixed overhead bridge elements use absolute Y/Z where the MJCF places
    them on ``x_carriage`` (Y rail at z = 3 m).

    Args:
        configuration: Joint / EE position ``(x, y, z)`` [m].

    Returns:
        List of body envelopes to sample for collision queries.
    """
    if configuration.shape != (3,):
        raise ValueError(f"Expected shape (3,), got {configuration.shape}")
    x, y, z = (
        float(configuration[0]),
        float(configuration[1]),
        float(configuration[2]),
    )

    return [
        # Z column and end-effector (``z_hoist`` body).
        BodyEnvelope(
            centre=np.array([x, y, z + 1.45], dtype=np.float64),
            half_extent=np.array([0.11, 0.11, 1.45], dtype=np.float64),
        ),
        BodyEnvelope(
            centre=np.array([x, y, z], dtype=np.float64),
            half_extent=np.array([0.28, 0.22, 0.08], dtype=np.float64),
        ),
        BodyEnvelope(
            centre=np.array([x, y, z - 0.06], dtype=np.float64),
            half_extent=np.array([0.30, 0.26, 0.04], dtype=np.float64),
        ),
        # Y trolley (``y_carriage`` at z = 3 m).
        BodyEnvelope(
            centre=np.array([x, y, 2.92], dtype=np.float64),
            half_extent=np.array([0.30, 0.22, 0.12], dtype=np.float64),
        ),
        BodyEnvelope(
            centre=np.array([x, y, 3.02], dtype=np.float64),
            half_extent=np.array([0.20, 0.18, 0.06], dtype=np.float64),
        ),
        # X bridge and rail carriages (``x_carriage`` at z = 3 m).
        BodyEnvelope(
            centre=np.array([x, 2.0, 2.95], dtype=np.float64),
            half_extent=np.array([0.16, 1.96, 0.08], dtype=np.float64),
        ),
        BodyEnvelope(
            centre=np.array([x, 0.28, 2.97], dtype=np.float64),
            half_extent=np.array([0.20, 0.14, 0.11], dtype=np.float64),
        ),
        BodyEnvelope(
            centre=np.array([x, 3.72, 2.97], dtype=np.float64),
            half_extent=np.array([0.20, 0.14, 0.11], dtype=np.float64),
        ),
    ]


def ppp_envelope_sample_points(
    configuration: npt.NDArray[np.float64],
    *,
    extra_envelopes: list[BodyEnvelope] | None = None,
) -> npt.NDArray[np.float64]:
    """Return 8-corner samples for all PPP body envelopes.

    Args:
        configuration: Joint / EE position ``(x, y, z)`` [m].
        extra_envelopes: Optional additional boxes (e.g. welded cargo).

    Returns:
        Point array, shape ``(N, 3)``.
    """
    envelopes = ppp_body_envelopes(configuration)
    if extra_envelopes:
        envelopes = envelopes + list(extra_envelopes)
    corners = [_box_corners(env.centre, env.half_extent) for env in envelopes]
    return np.vstack(corners)
