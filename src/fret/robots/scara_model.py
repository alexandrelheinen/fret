"""SCARA robot kinematic model for the ARCO-FRET planning pipeline.

Implements the R-R-P-R (4-DOF) SCARA kinematics described in
``src/fret/urdf/scara.xacro``.  All parameters are extracted directly
from the XACRO property declarations so that this model stays in sync
with the robot description.

Kinematic chain (DH-style, world origin = base_link origin):

  J1 (revolute, Z-axis) at z = H_BASE = 0.1665 m
    └─ Arm-0 link  (length L1 = 0.325 m, top face at z = H_BASE + T_ARM = 0.238 m)
       J2 (revolute, Z-axis) at [L1·cos(q1), L1·sin(q1), 0.238]
         └─ Arm-1 link  (length L2 = 0.275 m, same height as arm-0 top)
            J3 (prismatic, −Z-axis) at [x_J3, y_J3, 0.238]
              └─ Extension/quill  travels [0, 0, −q3] from J3
                 J4 (revolute, Z-axis, continuous) at [x_J3, y_J3, 0.238 − q3]
                   └─ End-effector plate (same xyz as J4)

End-effector position::

    x_ee = L1·cos(q1) + L2·cos(q1 + q2)
    y_ee = L1·sin(q1) + L2·sin(q1 + q2)
    z_ee = H_BASE + T_ARM − q3          (lower with more extension)
"""

from __future__ import annotations

import math

from fret.robots.robot_model import RobotModel

# ---------------------------------------------------------------------------
# Kinematic constants (from scara.xacro)
# ---------------------------------------------------------------------------
_H_BASE: float = 0.1665  # link_0_cylinder_height: J1 z in world frame (m)
_T_ARM: float = 0.0715  # link_1_height = link_2_height = j2_z (m)
_L1: float = 0.325  # arm_0_length: distance J1 → J2 (m)
_L2: float = 0.275  # arm_1_length: distance J2 → J3 (m)

# Height of J1 in world frame and height of the arm-link top faces
_Z_J1: float = _H_BASE  # 0.1665 m
_Z_J2: float = _H_BASE + _T_ARM  # 0.238 m (J2 and J3 share this height)

# ---------------------------------------------------------------------------
# Joint limits (from scara.xacro — PI = math.pi)
# ---------------------------------------------------------------------------
_J1_LOWER: float = -132.0 * math.pi / 180.0  # -2.3038 rad
_J1_UPPER: float = 132.0 * math.pi / 180.0  # +2.3038 rad
_J2_LOWER: float = -150.0 * math.pi / 180.0  # -2.6180 rad
_J2_UPPER: float = 150.0 * math.pi / 180.0  # +2.6180 rad
_J3_LOWER: float = 0.0  # 0 mm (fully retracted)
_J3_UPPER: float = 0.2  # 200 mm (fully extended)
# J4 is continuous; the planner needs finite bounds — use ±π.
_J4_LOWER: float = -math.pi
_J4_UPPER: float = math.pi

# Number of samples per arm link for collision checking.
_LINK_SAMPLES: int = 5


# ---------------------------------------------------------------------------
# Kinematic helpers
# ---------------------------------------------------------------------------


def _lerp(pa: list[float], pb: list[float], n: int) -> list[list[float]]:
    """Return n+1 linearly interpolated points from *pa* to *pb*.

    Args:
        pa: Start point [x, y, z].
        pb: End point [x, y, z].
        n: Number of intervals (yields n+1 points including both endpoints).

    Returns:
        List of n+1 ``[x, y, z]`` triples.
    """
    return [
        [
            pa[0] + (pb[0] - pa[0]) * i / n,
            pa[1] + (pb[1] - pa[1]) * i / n,
            pa[2] + (pb[2] - pa[2]) * i / n,
        ]
        for i in range(n + 1)
    ]


class ScaraModel(RobotModel):
    """Kinematic model for the FRET SCARA (R-R-P-R) manipulator.

    Uses the forward-kinematics formulation from the XACRO description to
    compute a set of Cartesian control points distributed along the entire
    kinematic chain.  The control points are fed to an
    :class:`~fret.perception.OccupancyAdapter` by
    :meth:`~fret.robots.RobotModel.make_state_validator`.

    Collision-relevant geometry:
        - Arm links (arm-0, arm-1) travel at z ≥ 0.17 m, safely above the
          0.10 m-tall floor obstacles in the ARCO scenario.
        - The quill extension descends to z_ee = H_BASE + T_ARM − q3_max
          ≈ 0.038 m at full travel, so it is the primary collision risk.
        - All three links are sampled to cover future taller obstacles.
    """

    @property
    def n_joints(self) -> int:
        """Return the number of joints (4 for R-R-P-R SCARA)."""
        return 4

    @property
    def joint_limits(self) -> list[tuple[float, float]]:
        """Return joint position limits (lower, upper) for all four joints.

        Returns:
            [(J1_lower, J1_upper), (J2_lower, J2_upper),
             (J3_lower, J3_upper), (J4_lower, J4_upper)]
            where angles are in radians and J3 is in metres.
        """
        return [
            (_J1_LOWER, _J1_UPPER),
            (_J2_LOWER, _J2_UPPER),
            (_J3_LOWER, _J3_UPPER),
            (_J4_LOWER, _J4_UPPER),
        ]

    def forward_kinematics(self, q: list[float]) -> list[list[float]]:
        """Compute Cartesian control points for SCARA configuration *q*.

        Samples :data:`_LINK_SAMPLES` + 1 points along each of:
        arm-0 (J1 → J2), arm-1 (J2 → J3), and the quill extension
        (J3 → end-effector).

        Args:
            q: Joint configuration [q1, q2, q3, q4] where q1, q2, q4 are
               revolute angles (rad) and q3 is the prismatic extension (m).

        Returns:
            List of ``[x, y, z]`` control points (metres, world frame).
            Guaranteed to contain at least the end-effector position.

        Raises:
            ValueError: If ``len(q) != 4``.
        """
        if len(q) != 4:
            raise ValueError(
                f"ScaraModel expects 4 joints, got {len(q)}"
            )

        q1, q2, q3, _ = q  # q4 is the continuous tool-rotation angle

        # ── Joint positions ──────────────────────────────────────────
        # J1 is at the base, z = _Z_J1
        x_j1, y_j1, z_j1 = 0.0, 0.0, _Z_J1

        # J2: end of arm-0, rotated by q1 about Z
        x_j2 = _L1 * math.cos(q1)
        y_j2 = _L1 * math.sin(q1)
        z_j2 = _Z_J2  # arm-0 top face height

        # J3: end of arm-1, rotated by (q1 + q2) about Z from J2
        x_j3 = x_j2 + _L2 * math.cos(q1 + q2)
        y_j3 = y_j2 + _L2 * math.sin(q1 + q2)
        z_j3 = _Z_J2  # J3 is at the same height as J2

        # End-effector: J3 + quill extension in −Z direction
        x_ee = x_j3
        y_ee = y_j3
        z_ee = _Z_J2 - q3

        # ── Sample control points ────────────────────────────────────
        points: list[list[float]] = []

        # Arm-0 (J1 → J2) — samples at uniform intervals along the link
        points.extend(
            _lerp([x_j1, y_j1, z_j1], [x_j2, y_j2, z_j2], _LINK_SAMPLES)
        )

        # Arm-1 (J2 → J3) — skip J2 duplicate
        points.extend(
            _lerp([x_j2, y_j2, z_j2], [x_j3, y_j3, z_j3], _LINK_SAMPLES)[
                1:
            ]
        )

        # Quill extension (J3 → end-effector) — skip J3 duplicate
        points.extend(
            _lerp(
                [x_j3, y_j3, z_j3], [x_ee, y_ee, z_ee], _LINK_SAMPLES
            )[1:]
        )

        return points
