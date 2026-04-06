"""Tests for model-specific controller parameters in sim.py.

Validates:
- SCARA uses SCARA joint names and end_effector_link.
- Gazebo bridge remaps joint states directly to /joint_states.
- Spawn rotates the robot 180° so the arm faces away from obstacles.

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
    """Validate joint-state pipeline wiring and spawn pose in sim.py."""

    def _source(self):
        sim_py = os.path.join(_SRC_DIR, "fret", "launch", "sim.py")
        with open(sim_py, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_bridge_remaps_directly_to_joint_states(self):
        """Gazebo bridge must remap joint states straight to /joint_states.

        Gazebo Harmonic's JointStatePublisher system plugin (loaded via the
        <gazebo> block in scara.xacro) publishes to the world-scoped topic
        /world/<world_name>/model/<model>/joint_state.  The bridge must use
        this world-scoped topic and remap it to /joint_states so
        robot_state_publisher can publish TF.
        """
        source = self._source()
        self.assertIn('"/joint_states"', source)
        self.assertNotIn('"joint_state_publisher"', source)
        # Bridge must use world-scoped topic (not bare /model/<model>/…)
        self.assertIn("world_name", source)
        self.assertIn("/world/", source)

    def test_spawn_rotated_180_degrees(self):
        """Robot must spawn with 180° yaw so the arm faces away from obstacles."""
        source = self._source()
        self.assertIn('"-Y"', source)
        self.assertIn('"3.14159"', source)

    def test_perception_bridge_included(self):
        """sim.py must launch the perception_bridge node."""
        source = self._source()
        self.assertIn("perception_bridge", source)


if __name__ == "__main__":
    unittest.main()


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
    """Validate joint-state pipeline wiring and spawn pose in sim.py."""

    def _source(self):
        sim_py = os.path.join(_SRC_DIR, "fret", "launch", "sim.py")
        with open(sim_py, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_bridge_remaps_directly_to_joint_states(self):
        """Gazebo bridge must remap joint states straight to /joint_states.

        Gazebo Harmonic's JointStatePublisher system plugin (loaded via the
        <gazebo> block in ur.xacro) publishes to the world-scoped topic
        /world/<world_name>/model/<model>/joint_state.  The bridge must use
        this world-scoped topic and remap it to /joint_states so
        robot_state_publisher can publish TF.
        """
        source = self._source()
        self.assertIn('"/joint_states"', source)
        self.assertNotIn('"joint_state_publisher"', source)
        # Bridge must use world-scoped topic (not bare /model/<model>/…)
        self.assertIn("world_name", source)
        self.assertIn("/world/", source)

    def test_spawn_rotated_180_degrees(self):
        """Robot must spawn with 180° yaw so the arm faces away from obstacles."""
        source = self._source()
        self.assertIn('"-Y"', source)
        self.assertIn('"3.14159"', source)


if __name__ == "__main__":
    unittest.main()
