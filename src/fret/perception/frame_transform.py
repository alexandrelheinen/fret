"""Frame transformation utility for the FRET perception pipeline.

Provides a pure-Python ``FrameTransform`` class that applies a 4×4
homogeneous transformation matrix to lists of 3-D points.  This is used
to convert raw sensor-frame point clouds into the canonical ``world``
frame required by the ARCO-FRET integration contract
(``docs/arco/spec-frames-and-units.md``).

No ROS 2 runtime dependency; the class can be constructed from any
rotation/translation source and tested independently.
"""

from __future__ import annotations

import math
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

Point = List[float]  # [x, y, z]
Matrix4x4 = List[List[float]]  # row-major 4×4 matrix


class FrameTransform:
    """Apply a rigid-body transform to a list of 3-D points.

    The transform is represented as a 4×4 homogeneous matrix in
    row-major order::

        | r00 r01 r02 tx |
        | r10 r11 r12 ty |
        | r20 r21 r22 tz |
        |  0   0   0   1 |

    Args:
        matrix: 4×4 homogeneous transformation matrix given as a list of
            four rows, each a list of four floats.

    Raises:
        ValueError: If the matrix does not have shape 4×4.

    Example::

        tf = FrameTransform.from_translation(1.0, 0.0, 0.5)
        world_pts = tf.transform_points(sensor_pts)
    """

    def __init__(self, matrix: Matrix4x4) -> None:
        if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError(
                "matrix must be a 4×4 list of lists, "
                f"got shape {len(matrix)}×{len(matrix[0]) if matrix else 0}"
            )
        self._matrix: Matrix4x4 = [list(row) for row in matrix]

    # ------------------------------------------------------------------
    # Class-method constructors
    # ------------------------------------------------------------------

    @classmethod
    def identity(cls) -> FrameTransform:
        """Create the identity (no-op) transform.

        Returns:
            A :class:`FrameTransform` that leaves all points unchanged.
        """
        return cls(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    @classmethod
    def from_translation(
        cls, tx: float, ty: float, tz: float
    ) -> FrameTransform:
        """Create a pure-translation transform.

        Args:
            tx: Translation along the X axis (meters).
            ty: Translation along the Y axis (meters).
            tz: Translation along the Z axis (meters).

        Returns:
            A :class:`FrameTransform` that shifts all points by
            ``(tx, ty, tz)``.
        """
        return cls(
            [
                [1.0, 0.0, 0.0, tx],
                [0.0, 1.0, 0.0, ty],
                [0.0, 0.0, 1.0, tz],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    @classmethod
    def from_rotation_z(cls, angle: float) -> FrameTransform:
        """Create a rotation transform about the Z axis.

        Args:
            angle: Rotation angle in radians (right-hand rule about +Z).

        Returns:
            A :class:`FrameTransform` that rotates all points about the
            Z axis by ``angle``.
        """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return cls(
            [
                [cos_a, -sin_a, 0.0, 0.0],
                [sin_a, cos_a, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    @classmethod
    def from_rotation_matrix_and_translation(
        cls,
        rotation: List[List[float]],
        translation: Tuple[float, float, float],
    ) -> FrameTransform:
        """Create a transform from a 3×3 rotation matrix and a translation.

        Args:
            rotation: 3×3 rotation matrix as a list of three rows, each
                containing three floats.
            translation: Translation vector ``(tx, ty, tz)`` in meters.

        Returns:
            A :class:`FrameTransform` combining the given rotation and
            translation.

        Raises:
            ValueError: If ``rotation`` is not a 3×3 matrix.
        """
        if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
            raise ValueError(
                "rotation must be a 3×3 list of lists, "
                f"got shape {len(rotation)}×"
                f"{len(rotation[0]) if rotation else 0}"
            )
        tx, ty, tz = translation
        matrix = [
            [rotation[0][0], rotation[0][1], rotation[0][2], tx],
            [rotation[1][0], rotation[1][1], rotation[1][2], ty],
            [rotation[2][0], rotation[2][1], rotation[2][2], tz],
            [0.0, 0.0, 0.0, 1.0],
        ]
        return cls(matrix)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def matrix(self) -> Matrix4x4:
        """The underlying 4×4 homogeneous matrix (copy)."""
        return [list(row) for row in self._matrix]

    # ------------------------------------------------------------------
    # Transform operations
    # ------------------------------------------------------------------

    def transform_points(self, points: List[Point]) -> List[Point]:
        """Apply the transform to a list of 3-D points.

        Each input point ``[x, y, z]`` is treated as a homogeneous
        column vector ``[x, y, z, 1]^T``; the rotational and
        translational components of the matrix are applied and the
        resulting ``[x', y', z']`` is returned.

        Args:
            points: List of ``[x, y, z]`` triples in the source frame
                (meters).

        Returns:
            New list of ``[x', y', z']`` triples expressed in the target
            frame (meters).  The input list is not modified.
        """
        m = self._matrix
        result: List[Point] = []
        for pt in points:
            x, y, z = pt[0], pt[1], pt[2]
            nx = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]
            ny = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]
            nz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]
            result.append([nx, ny, nz])
        return result

    def compose(self, other: FrameTransform) -> FrameTransform:
        """Compose this transform with another, returning a new transform.

        The resulting transform is equivalent to applying ``other`` first,
        then ``self``::

            composed.transform_points(pts) ==
                self.transform_points(other.transform_points(pts))

        Args:
            other: The transform to apply first.

        Returns:
            A new :class:`FrameTransform` representing the composition
            ``self ∘ other``.
        """
        a = self._matrix
        b = other._matrix
        result = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                result[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
        return FrameTransform(result)
