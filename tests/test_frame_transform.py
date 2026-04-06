"""Unit tests for FrameTransform (Issue 03: Point Cloud Acquisition).

Validates:
- Identity transform leaves points unchanged.
- Pure-translation constructor and application.
- Z-axis rotation constructor and application.
- Full rotation-matrix + translation constructor.
- Composition of two transforms.
- Input list immutability.
- Constructor validation for malformed matrices.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Make the fret package importable when running tests without a full ROS build
# (e.g., locally with ``python3 -m unittest discover tests``).
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.frame_transform import FrameTransform

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

_TOL = 1e-9  # floating-point tolerance for coordinate comparisons


def _pt(x, y, z):
    """Return a point as a list."""
    return [float(x), float(y), float(z)]


def _assert_points_close(test_case, actual, expected, tol=_TOL):
    """Assert that two lists of points are element-wise close."""
    test_case.assertEqual(
        len(actual),
        len(expected),
        f"Point list length mismatch: {len(actual)} != {len(expected)}",
    )
    for i, (a, e) in enumerate(zip(actual, expected)):
        for j in range(3):
            test_case.assertAlmostEqual(
                a[j],
                e[j],
                delta=tol,
                msg=(
                    f"Point {i} component {j} mismatch: "
                    f"{a[j]:.10f} != {e[j]:.10f}"
                ),
            )


# ---------------------------------------------------------------------------
# Constructor validation tests
# ---------------------------------------------------------------------------


class TestFrameTransformConstructor(unittest.TestCase):
    """Validate FrameTransform constructor argument checking."""

    def test_valid_4x4_matrix_succeeds(self):
        """A well-formed 4×4 matrix must be accepted."""
        mat = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        tf = FrameTransform(mat)
        self.assertEqual(tf.matrix, mat)

    def test_wrong_row_count_raises(self):
        """A matrix with fewer than 4 rows must raise ValueError."""
        mat = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
        with self.assertRaises(ValueError):
            FrameTransform(mat)

    def test_wrong_column_count_raises(self):
        """A matrix with rows shorter than 4 must raise ValueError."""
        mat = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
        ]
        with self.assertRaises(ValueError):
            FrameTransform(mat)

    def test_matrix_property_returns_copy(self):
        """matrix property must return a copy, not a live reference."""
        tf = FrameTransform.identity()
        mat_copy = tf.matrix
        mat_copy[0][0] = 99.0
        # Original must be unaffected
        self.assertEqual(tf.matrix[0][0], 1.0)


# ---------------------------------------------------------------------------
# Identity transform tests
# ---------------------------------------------------------------------------


class TestIdentityTransform(unittest.TestCase):
    """Unit tests for FrameTransform.identity()."""

    def setUp(self):
        self.tf = FrameTransform.identity()

    def test_identity_matrix_diagonal(self):
        """Identity matrix must have 1.0 on the diagonal."""
        mat = self.tf.matrix
        for i in range(4):
            self.assertAlmostEqual(mat[i][i], 1.0)

    def test_identity_matrix_off_diagonal(self):
        """Identity matrix must have 0.0 off the diagonal."""
        mat = self.tf.matrix
        for i in range(4):
            for j in range(4):
                if i != j:
                    self.assertAlmostEqual(mat[i][j], 0.0)

    def test_identity_leaves_point_unchanged(self):
        """Identity transform must leave a single point unchanged."""
        pts = [_pt(1.0, 2.0, 3.0)]
        result = self.tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 2.0, 3.0)])

    def test_identity_leaves_multiple_points_unchanged(self):
        """Identity transform must leave multiple points unchanged."""
        pts = [_pt(0.0, 0.0, 0.0), _pt(1.5, -2.0, 3.0), _pt(-1.0, 0.5, 2.5)]
        result = self.tf.transform_points(pts)
        _assert_points_close(self, result, pts)

    def test_identity_on_empty_list(self):
        """Identity transform applied to an empty list must return empty."""
        result = self.tf.transform_points([])
        self.assertEqual(result, [])


# ---------------------------------------------------------------------------
# Translation transform tests
# ---------------------------------------------------------------------------


class TestFromTranslation(unittest.TestCase):
    """Unit tests for FrameTransform.from_translation()."""

    def test_translation_shifts_point_correctly(self):
        """from_translation must shift each point by (tx, ty, tz)."""
        tf = FrameTransform.from_translation(1.0, 2.0, 3.0)
        pts = [_pt(0.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 2.0, 3.0)])

    def test_translation_zero_is_identity(self):
        """Zero translation must behave like identity."""
        tf = FrameTransform.from_translation(0.0, 0.0, 0.0)
        pts = [_pt(5.0, -3.0, 2.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, pts)

    def test_translation_negative_values(self):
        """Negative translations must shift points in the negative direction."""
        tf = FrameTransform.from_translation(-1.0, -2.0, -3.0)
        pts = [_pt(1.0, 2.0, 3.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(0.0, 0.0, 0.0)])

    def test_translation_multiple_points(self):
        """Translation must be applied identically to all points."""
        tf = FrameTransform.from_translation(0.5, 0.5, 0.5)
        pts = [_pt(0.0, 0.0, 0.0), _pt(1.0, 1.0, 1.0)]
        result = tf.transform_points(pts)
        _assert_points_close(
            self, result, [_pt(0.5, 0.5, 0.5), _pt(1.5, 1.5, 1.5)]
        )

    def test_translation_x_axis_only(self):
        """Translation along X only must not affect Y or Z."""
        tf = FrameTransform.from_translation(3.0, 0.0, 0.0)
        pts = [_pt(0.0, 4.0, 5.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(3.0, 4.0, 5.0)])


# ---------------------------------------------------------------------------
# Rotation (Z-axis) transform tests
# ---------------------------------------------------------------------------


class TestFromRotationZ(unittest.TestCase):
    """Unit tests for FrameTransform.from_rotation_z()."""

    def test_rotation_zero_is_identity(self):
        """Zero rotation about Z must behave like identity."""
        tf = FrameTransform.from_rotation_z(0.0)
        pts = [_pt(1.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 0.0, 0.0)])

    def test_rotation_90_degrees_x_to_y(self):
        """+90° rotation about Z must map +X point to +Y."""
        tf = FrameTransform.from_rotation_z(math.pi / 2)
        pts = [_pt(1.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(0.0, 1.0, 0.0)])

    def test_rotation_180_degrees_inverts_x(self):
        """180° rotation about Z must negate X and Y."""
        tf = FrameTransform.from_rotation_z(math.pi)
        pts = [_pt(1.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(-1.0, 0.0, 0.0)])

    def test_rotation_does_not_affect_z(self):
        """Rotation about Z must leave the Z component unchanged."""
        tf = FrameTransform.from_rotation_z(math.pi / 4)
        pts = [_pt(1.0, 0.0, 7.0)]
        result = tf.transform_points(pts)
        self.assertAlmostEqual(result[0][2], 7.0, delta=_TOL)

    def test_rotation_minus_90_degrees_y_to_x(self):
        """-90° rotation about Z must map +Y point to +X."""
        tf = FrameTransform.from_rotation_z(-math.pi / 2)
        pts = [_pt(0.0, 1.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 0.0, 0.0)])


# ---------------------------------------------------------------------------
# from_rotation_matrix_and_translation tests
# ---------------------------------------------------------------------------


class TestFromRotationMatrixAndTranslation(unittest.TestCase):
    """Unit tests for FrameTransform.from_rotation_matrix_and_translation."""

    def test_identity_rotation_and_zero_translation(self):
        """Identity rotation + zero translation must behave like identity."""
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        tf = FrameTransform.from_rotation_matrix_and_translation(
            R, (0.0, 0.0, 0.0)
        )
        pts = [_pt(1.0, 2.0, 3.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, pts)

    def test_identity_rotation_with_translation(self):
        """Identity rotation + translation must purely shift the point."""
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        tf = FrameTransform.from_rotation_matrix_and_translation(
            R, (5.0, -3.0, 1.0)
        )
        pts = [_pt(0.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(5.0, -3.0, 1.0)])

    def test_invalid_rotation_matrix_raises(self):
        """A rotation matrix with wrong shape must raise ValueError."""
        R = [[1.0, 0.0], [0.0, 1.0]]  # 2×2 instead of 3×3
        with self.assertRaises(ValueError):
            FrameTransform.from_rotation_matrix_and_translation(
                R, (0.0, 0.0, 0.0)
            )

    def test_rotation_combined_with_translation(self):
        """Rotation followed by translation must produce the correct result."""
        # 90° about Z, then translate by (1, 0, 0)
        cos90 = math.cos(math.pi / 2)
        sin90 = math.sin(math.pi / 2)
        R = [
            [cos90, -sin90, 0.0],
            [sin90, cos90, 0.0],
            [0.0, 0.0, 1.0],
        ]
        tf = FrameTransform.from_rotation_matrix_and_translation(
            R, (1.0, 0.0, 0.0)
        )
        pts = [_pt(1.0, 0.0, 0.0)]
        # Rotation maps (1, 0, 0) → (0, 1, 0), then translate: (1, 1, 0)
        result = tf.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 1.0, 0.0)])


# ---------------------------------------------------------------------------
# Compose tests
# ---------------------------------------------------------------------------


class TestCompose(unittest.TestCase):
    """Unit tests for FrameTransform.compose()."""

    def test_compose_two_translations(self):
        """Composing two translations must sum the translation vectors."""
        tf1 = FrameTransform.from_translation(1.0, 0.0, 0.0)
        tf2 = FrameTransform.from_translation(0.0, 2.0, 0.0)
        composed = tf1.compose(tf2)
        pts = [_pt(0.0, 0.0, 0.0)]
        result = composed.transform_points(pts)
        _assert_points_close(self, result, [_pt(1.0, 2.0, 0.0)])

    def test_compose_identity_is_no_op(self):
        """Composing with identity must return the same transform."""
        tf = FrameTransform.from_translation(3.0, 1.0, -2.0)
        composed = tf.compose(FrameTransform.identity())
        pts = [_pt(0.0, 0.0, 0.0)]
        result_composed = composed.transform_points(pts)
        result_original = tf.transform_points(pts)
        _assert_points_close(self, result_composed, result_original)

    def test_compose_rotation_then_translation(self):
        """Rotation-then-translation composition must match sequential apply."""
        tf_rot = FrameTransform.from_rotation_z(math.pi / 2)
        tf_trans = FrameTransform.from_translation(1.0, 0.0, 0.0)
        # tf_trans applied after tf_rot
        composed = tf_trans.compose(tf_rot)
        pts = [_pt(1.0, 0.0, 0.0)]
        result_composed = composed.transform_points(pts)
        # Manual: rotate first (1,0,0) → (0,1,0), then translate → (1,1,0)
        result_manual = tf_trans.transform_points(tf_rot.transform_points(pts))
        _assert_points_close(self, result_composed, result_manual)

    def test_compose_returns_new_instance(self):
        """compose() must return a new FrameTransform instance."""
        tf1 = FrameTransform.identity()
        tf2 = FrameTransform.identity()
        composed = tf1.compose(tf2)
        self.assertIsNot(composed, tf1)
        self.assertIsNot(composed, tf2)


# ---------------------------------------------------------------------------
# Input immutability tests
# ---------------------------------------------------------------------------


class TestInputImmutability(unittest.TestCase):
    """Verify that transform_points does not mutate input data."""

    def test_transform_does_not_modify_input(self):
        """transform_points must not modify the original point list."""
        tf = FrameTransform.from_translation(10.0, 10.0, 10.0)
        pts = [_pt(1.0, 2.0, 3.0), _pt(4.0, 5.0, 6.0)]
        original = [list(p) for p in pts]
        tf.transform_points(pts)
        self.assertEqual(pts, original)

    def test_constructor_does_not_alias_input_matrix(self):
        """Modifying the input matrix after construction must not affect tf."""
        mat = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        tf = FrameTransform(mat)
        mat[0][3] = 99.0  # mutate after construction
        pts = [_pt(0.0, 0.0, 0.0)]
        result = tf.transform_points(pts)
        # Should still behave as identity (translation = 0)
        _assert_points_close(self, result, [_pt(0.0, 0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
