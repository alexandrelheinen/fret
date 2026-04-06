"""Unit tests for the SCARA robot kinematic model.

Validates:
- ScaraModel forward kinematics at known configurations.
- Joint limits match the scara.xacro parameter declarations.
- Control-point list structure and dimensions.
- State-validator wiring via RobotModel base class.

All tests are pure Python; no ROS 2 runtime is required.
"""

import math
import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")

if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.robots.robot_model import RobotModel
from fret.robots.scara_model import (
    ScaraModel,
    _H_BASE,
    _J1_LOWER,
    _J1_UPPER,
    _J2_LOWER,
    _J2_UPPER,
    _J3_LOWER,
    _J3_UPPER,
    _L1,
    _L2,
    _LINK_SAMPLES,
    _T_ARM,
)

# ---------------------------------------------------------------------------
# Constants (should match scara.xacro)
# ---------------------------------------------------------------------------
_EPS = 1e-9  # floating-point comparison tolerance
_Z_J2 = _H_BASE + _T_ARM  # 0.1665 + 0.0715 = 0.238 m


class TestScaraModelInterface(unittest.TestCase):
    """ScaraModel must satisfy the RobotModel abstract interface."""

    def setUp(self):
        self.model = ScaraModel()

    def test_is_robot_model_subclass(self):
        """ScaraModel must inherit from RobotModel."""
        self.assertIsInstance(self.model, RobotModel)

    def test_n_joints(self):
        """SCARA has 4 DOF."""
        self.assertEqual(self.model.n_joints, 4)

    def test_joint_limits_length(self):
        """joint_limits must return exactly 4 (lower, upper) pairs."""
        limits = self.model.joint_limits
        self.assertEqual(len(limits), 4)

    def test_joint_limits_structure(self):
        """Each entry in joint_limits must be (lower, upper) with lower < upper."""
        for i, (lower, upper) in enumerate(self.model.joint_limits):
            with self.subTest(joint=i):
                self.assertLess(lower, upper)

    def test_joint_limits_values(self):
        """Joint limits must match scara.xacro parameter declarations."""
        limits = self.model.joint_limits
        self.assertAlmostEqual(limits[0][0], _J1_LOWER, places=6)
        self.assertAlmostEqual(limits[0][1], _J1_UPPER, places=6)
        self.assertAlmostEqual(limits[1][0], _J2_LOWER, places=6)
        self.assertAlmostEqual(limits[1][1], _J2_UPPER, places=6)
        self.assertAlmostEqual(limits[2][0], _J3_LOWER, places=6)
        self.assertAlmostEqual(limits[2][1], _J3_UPPER, places=6)


class TestScaraModelForwardKinematics(unittest.TestCase):
    """Forward kinematics correctness tests."""

    def setUp(self):
        self.model = ScaraModel()

    def _ee(self, q):
        """Return the last control point (end-effector position)."""
        points = self.model.forward_kinematics(q)
        return points[-1]

    # -- Zero configuration --------------------------------------------------

    def test_zero_config_ee_on_positive_x_axis(self):
        """At q=[0,0,0,0] EE must lie on the +X axis at reach L1+L2."""
        ee = self._ee([0.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(ee[0], _L1 + _L2, places=6)
        self.assertAlmostEqual(ee[1], 0.0, places=6)

    def test_zero_config_ee_height(self):
        """At q3=0 (retracted) EE height must equal H_BASE + T_ARM."""
        ee = self._ee([0.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(ee[2], _Z_J2, places=6)

    def test_full_extension_ee_height(self):
        """At q3=J3_UPPER EE height must equal H_BASE + T_ARM - J3_UPPER."""
        q3_max = _J3_UPPER
        ee = self._ee([0.0, 0.0, q3_max, 0.0])
        expected_z = _Z_J2 - q3_max
        self.assertAlmostEqual(ee[2], expected_z, places=6)

    # -- Known configurations ------------------------------------------------

    def test_q1_90deg_reach(self):
        """At q1=π/2, q2=0 the arm points in +Y direction."""
        q1 = math.pi / 2.0
        ee = self._ee([q1, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(ee[0], 0.0, places=6)
        self.assertAlmostEqual(ee[1], _L1 + _L2, places=6)

    def test_q2_180deg_folds_arm(self):
        """At q1=0, q2=π the arm folds back; x_ee = L1 - L2."""
        q2 = math.pi
        ee = self._ee([0.0, q2, 0.0, 0.0])
        self.assertAlmostEqual(ee[0], _L1 - _L2, places=6)
        self.assertAlmostEqual(ee[1], 0.0, places=6)

    def test_j4_does_not_affect_position(self):
        """Tool rotation (q4) must not change the EE Cartesian position."""
        q_base = [0.3, -0.5, 0.1, 0.0]
        pts_a = self.model.forward_kinematics(q_base)
        q_rotated = [0.3, -0.5, 0.1, math.pi / 3.0]
        pts_b = self.model.forward_kinematics(q_rotated)

        self.assertEqual(len(pts_a), len(pts_b))
        for i, (pa, pb) in enumerate(zip(pts_a, pts_b)):
            with self.subTest(point=i):
                self.assertAlmostEqual(pa[0], pb[0], places=6)
                self.assertAlmostEqual(pa[1], pb[1], places=6)
                self.assertAlmostEqual(pa[2], pb[2], places=6)

    # -- Control point count / structure ------------------------------------

    def test_forward_kinematics_returns_list(self):
        """forward_kinematics must return a non-empty list."""
        pts = self.model.forward_kinematics([0.0, 0.0, 0.0, 0.0])
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 0)

    def test_each_control_point_is_xyz_triple(self):
        """Every control point must be a list of exactly 3 floats."""
        pts = self.model.forward_kinematics([0.1, -0.2, 0.05, 0.3])
        for i, pt in enumerate(pts):
            with self.subTest(point=i):
                self.assertEqual(len(pt), 3)

    def test_control_point_count(self):
        """Control-point count must equal the expected sampled-chain length."""
        # arm-0: LINK_SAMPLES+1 points
        # arm-1: LINK_SAMPLES points (J2 duplicate skipped)
        # quill:  LINK_SAMPLES points (J3 duplicate skipped)
        expected = (_LINK_SAMPLES + 1) + _LINK_SAMPLES + _LINK_SAMPLES
        pts = self.model.forward_kinematics([0.0, 0.0, 0.0, 0.0])
        self.assertEqual(len(pts), expected)

    # -- Error handling ------------------------------------------------------

    def test_wrong_joint_count_raises(self):
        """forward_kinematics must raise ValueError for wrong joint count."""
        with self.assertRaises(ValueError):
            self.model.forward_kinematics([0.0, 0.0, 0.0])

        with self.assertRaises(ValueError):
            self.model.forward_kinematics([0.0, 0.0, 0.0, 0.0, 0.0])


class TestScaraModelStateValidator(unittest.TestCase):
    """State-validator factory tests via RobotModel.make_state_validator."""

    class _MockOccupancy:
        """Minimal OccupancyAdapter stub."""

        def __init__(self, occupied_threshold=1.0):
            self.occupied_threshold = occupied_threshold
            self.calls = []

        def is_free(self, point):
            self.calls.append(point)
            # Mark occupied if x > threshold (used for deterministic testing)
            return point[0] <= self.occupied_threshold

    def setUp(self):
        self.model = ScaraModel()

    def test_validator_returns_callable(self):
        """make_state_validator must return a callable."""
        occ = self._MockOccupancy()
        validator = self.model.make_state_validator(occ)
        self.assertTrue(callable(validator))

    def test_validator_free_space(self):
        """Validator returns True when all control points are free."""
        occ = self._MockOccupancy(occupied_threshold=10.0)  # all free
        validator = self.model.make_state_validator(occ)
        # Zero config: arm points along +X, all x <= L1+L2 ≈ 0.6
        self.assertTrue(validator([0.0, 0.0, 0.0, 0.0]))

    def test_validator_occupied_space(self):
        """Validator returns False when at least one control point is occupied."""
        occ = self._MockOccupancy(occupied_threshold=-0.1)  # all occupied
        validator = self.model.make_state_validator(occ)
        self.assertFalse(validator([0.0, 0.0, 0.0, 0.0]))

    def test_validator_calls_is_free_for_each_point(self):
        """Validator must query occupancy for every control point."""
        occ = self._MockOccupancy(occupied_threshold=10.0)
        validator = self.model.make_state_validator(occ)
        pts = self.model.forward_kinematics([0.0, 0.0, 0.0, 0.0])
        validator([0.0, 0.0, 0.0, 0.0])
        self.assertEqual(len(occ.calls), len(pts))


class TestScaraModelKinematicParameters(unittest.TestCase):
    """Sanity checks for the module-level kinematic constants."""

    def test_h_base_positive(self):
        """Base height must be positive."""
        self.assertGreater(_H_BASE, 0.0)

    def test_l1_greater_than_l2(self):
        """Arm-0 is longer than arm-1 for SCARA (L1 > L2)."""
        self.assertGreater(_L1, _L2)

    def test_z_j2_above_h_base(self):
        """Arm-0 top face must be above J1."""
        self.assertGreater(_Z_J2, _H_BASE)

    def test_j3_lower_is_zero(self):
        """Extension lower bound is 0 m (fully retracted)."""
        self.assertAlmostEqual(_J3_LOWER, 0.0, places=9)

    def test_j3_upper_is_200mm(self):
        """Extension upper bound is 0.2 m (200 mm)."""
        self.assertAlmostEqual(_J3_UPPER, 0.2, places=9)

    def test_min_ee_height_above_floor(self):
        """Minimum EE height (full extension) must be above the floor (z > 0)."""
        min_z = _Z_J2 - _J3_UPPER
        self.assertGreater(min_z, 0.0)


if __name__ == "__main__":
    unittest.main()
