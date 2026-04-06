"""Tests for model-specific controller parameters in sim.py.

Validates:
- UR models use UR joint names and tool0 as end-effector frame.
- SCARA uses SCARA joint names and end_effector_link.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Paths resolved relative to the repository root
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")

# Make the fret package importable without a full ROS 2 build
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.launch.sim import _controller_parameters_for_model


class TestSimLaunchControllerParameters(unittest.TestCase):
    """Validate model-aware controller parameter generation."""

    def test_ur3_uses_tool0_and_six_joints(self):
        """UR3 must use REP-compatible tool frame and 6-DOF joints."""
        params = _controller_parameters_for_model("ur3")

        self.assertEqual(params["robot_model"], "ur3")
        self.assertEqual(params["base_frame"], "base_link")
        self.assertEqual(params["ee_frame"], "tool0")
        self.assertEqual(
            params["joint_names"],
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

    def test_scara_uses_end_effector_link_and_four_joints(self):
        """SCARA must keep project-local frame and 4-DOF joint order."""
        params = _controller_parameters_for_model("scara")

        self.assertEqual(params["robot_model"], "scara")
        self.assertEqual(params["base_frame"], "base_link")
        self.assertEqual(params["ee_frame"], "end_effector_link")
        self.assertEqual(
            params["joint_names"],
            [
                "joint_arm_0",
                "joint_arm_1",
                "joint_extension",
                "joint_tool_rotate",
            ],
        )


class TestSimLaunchJointStatePipeline(unittest.TestCase):
    """Validate startup-resilient joint-state pipeline wiring in sim.py."""

    def test_sim_launch_uses_joint_state_publisher(self):
        """sim.py must include joint_state_publisher for stable /joint_states."""
        sim_py = os.path.join(_SRC_DIR, "fret", "launch", "sim.py")
        with open(sim_py, "r", encoding="utf-8") as fh:
            source = fh.read()

        self.assertIn("joint_state_publisher", source)
        self.assertIn("/sim_joint_states", source)


if __name__ == "__main__":
    unittest.main()
