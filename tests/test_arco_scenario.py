"""Tests for the ARCO obstacle scenario (Issue 02).

Validates:
- World SDF file existence and XML well-formedness.
- Presence of exactly the named obstacle and region models.
- All obstacle models are declared static (deterministic replay).
- Ground plane is present.
- arco_scenario.py launch file exists and references the correct world and
  default robot model.
- sim.py exposes the ``world`` launch argument.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import ast
import os
import unittest
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------------------
# Paths resolved relative to the repository root
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_SRC_FRET = os.path.join(_REPO_ROOT, "src", "fret")
_WORLD_FILE = os.path.join(_SRC_FRET, "worlds", "arco_scenario.sdf")
_SCENARIO_LAUNCH = os.path.join(_SRC_FRET, "launch", "arco_scenario.py")
_SIM_LAUNCH = os.path.join(_SRC_FRET, "launch", "sim.py")

# ---------------------------------------------------------------------------
# Contract constants (must match the world file and spec document)
# ---------------------------------------------------------------------------
EXPECTED_WORLD_NAME = "arco_scenario"

EXPECTED_OBSTACLE_NAMES = {
    "obstacle_box_a",
    "obstacle_box_b",
    "obstacle_box_c",
}

EXPECTED_REGION_NAMES = {
    "region_start",
    "region_target",
}

MIN_OBSTACLE_COUNT = 3


# ---------------------------------------------------------------------------
# World SDF tests
# ---------------------------------------------------------------------------


class TestArcoScenarioWorldFile(unittest.TestCase):
    """Structural validation of the ARCO scenario SDF world file."""

    def _parse(self):
        """Return (root, world) element pair; fail fast if file is missing."""
        root = ET.parse(_WORLD_FILE).getroot()
        world = root.find("world")
        self.assertIsNotNone(world, "<world> element missing from SDF root.")
        return root, world

    # -- file-level tests ----------------------------------------------------

    def test_world_file_exists(self):
        """World SDF file must be present in version control."""
        self.assertTrue(
            os.path.isfile(_WORLD_FILE),
            f"World file not found: {_WORLD_FILE}",
        )

    def test_world_file_is_valid_xml(self):
        """World SDF file must be well-formed XML."""
        root = ET.parse(_WORLD_FILE).getroot()
        self.assertEqual(root.tag, "sdf")

    def test_sdf_version_attribute(self):
        """SDF root element must carry a version attribute."""
        root = ET.parse(_WORLD_FILE).getroot()
        self.assertIn("version", root.attrib)

    # -- world-level tests ---------------------------------------------------

    def test_world_has_correct_name(self):
        """<world name="arco_scenario"> is required for stable entity IDs."""
        _, world = self._parse()
        self.assertEqual(world.get("name"), EXPECTED_WORLD_NAME)

    def test_ground_plane_present(self):
        """Ground plane model must exist for physical consistency."""
        _, world = self._parse()
        names = {m.get("name") for m in world.findall("model")}
        self.assertIn("ground_plane", names)

    # -- obstacle tests -------------------------------------------------------

    def test_all_obstacle_models_present(self):
        """Every obstacle named in the spec must appear in the world."""
        _, world = self._parse()
        names = {m.get("name") for m in world.findall("model")}
        for obstacle in EXPECTED_OBSTACLE_NAMES:
            with self.subTest(obstacle=obstacle):
                self.assertIn(
                    obstacle,
                    names,
                    f"Obstacle '{obstacle}' missing from world.",
                )

    def test_obstacle_count_at_least_three(self):
        """SCARA scenario requires at least three obstacle primitives."""
        _, world = self._parse()
        names = {m.get("name") for m in world.findall("model")}
        found = names & EXPECTED_OBSTACLE_NAMES
        self.assertGreaterEqual(
            len(found),
            MIN_OBSTACLE_COUNT,
            f"Only {len(found)} obstacles found; expected >= {MIN_OBSTACLE_COUNT}.",
        )

    def test_obstacle_models_are_static(self):
        """Obstacle models must be static for deterministic scenario replay."""
        _, world = self._parse()
        for model in world.findall("model"):
            name = model.get("name")
            if name not in EXPECTED_OBSTACLE_NAMES:
                continue
            static_elem = model.find("static")
            with self.subTest(model=name):
                self.assertIsNotNone(
                    static_elem,
                    f"Model '{name}' is missing <static>.",
                )
                self.assertEqual(
                    static_elem.text.strip(),
                    "true",
                    f"Model '{name}' must have <static>true</static>.",
                )

    # -- region marker tests -------------------------------------------------

    def test_region_markers_present(self):
        """Start and target region markers must be present in the world."""
        _, world = self._parse()
        names = {m.get("name") for m in world.findall("model")}
        for region in EXPECTED_REGION_NAMES:
            with self.subTest(region=region):
                self.assertIn(
                    region,
                    names,
                    f"Region marker '{region}' missing from world.",
                )

    def test_region_markers_have_no_collision(self):
        """Region markers are visual-only and must not block the robot."""
        _, world = self._parse()
        for model in world.findall("model"):
            if model.get("name") not in EXPECTED_REGION_NAMES:
                continue
            with self.subTest(region=model.get("name")):
                self.assertIsNone(
                    model.find(".//collision"),
                    f"Region '{model.get('name')}' must not have collision.",
                )


# ---------------------------------------------------------------------------
# Launch file tests
# ---------------------------------------------------------------------------


class TestArcoScenarioLaunchFile(unittest.TestCase):
    """Structural checks for the arco_scenario.py launch file."""

    def _read(self, path):
        with open(path, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_launch_file_exists(self):
        """arco_scenario.py must exist in the launch directory."""
        self.assertTrue(
            os.path.isfile(_SCENARIO_LAUNCH),
            f"Launch file not found: {_SCENARIO_LAUNCH}",
        )

    def test_launch_file_is_valid_python(self):
        """arco_scenario.py must be syntactically valid Python."""
        source = self._read(_SCENARIO_LAUNCH)
        try:
            ast.parse(source)
        except SyntaxError as exc:
            self.fail(f"arco_scenario.py has a syntax error: {exc}")

    def test_launch_file_references_arco_world(self):
        """arco_scenario.py must reference the arco_scenario world file."""
        source = self._read(_SCENARIO_LAUNCH)
        self.assertIn(
            "arco_scenario",
            source,
            "arco_scenario.py must reference the arco_scenario world.",
        )

    def test_launch_file_uses_scara_model(self):
        """arco_scenario.py must default to the scara robot model."""
        source = self._read(_SCENARIO_LAUNCH)
        self.assertIn(
            "scara",
            source,
            "arco_scenario.py must reference the scara robot model.",
        )

    def test_launch_file_includes_sim_launch(self):
        """arco_scenario.py must compose sim.py via IncludeLaunchDescription."""
        source = self._read(_SCENARIO_LAUNCH)
        self.assertIn("sim.py", source)
        self.assertIn("IncludeLaunchDescription", source)


class TestSimLaunchWorldArgument(unittest.TestCase):
    """Verify that sim.py exposes the new ``world`` launch argument."""

    def _read(self):
        with open(_SIM_LAUNCH, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_world_argument_declared(self):
        """sim.py must declare a 'world' launch argument."""
        source = self._read()
        self.assertIn(
            '"world"',
            source,
            "sim.py must declare a 'world' launch argument.",
        )

    def test_world_default_is_arco_scenario(self):
        """'world' argument default must use arco_scenario.sdf."""
        source = self._read()
        self.assertIn(
            "arco_scenario.sdf",
            source,
            "Default world must be arco_scenario.sdf.",
        )

    def test_sim_does_not_bridge_pose_v_to_tf(self):
        """sim.py must NOT have a ros_gz_bridge entry for gz.msgs.Pose_V.

        The Pose_V message from Gazebo's SceneBroadcaster includes the world
        entity with an empty name, which ros_gz_bridge converts to
        TransformStamped messages with frame_id="" and child_frame_id="".
        TF2 rejects these with TF_SELF_TRANSFORM / TF_NO_FRAME_ID errors that
        flood the log.  Obstacle TF is published as static transforms instead.
        """
        source = self._read()
        # The bridge argument string contains the bridged type as "[gz.msgs.Pose_V"
        # (with a leading square bracket as ros_gz_bridge direction indicator).
        self.assertNotIn(
            "[gz.msgs.Pose_V",
            source,
            "sim.py must not have a bridge entry for gz.msgs.Pose_V "
            "(causes empty-frame TF spam).",
        )
        # The pose/info topic must not appear in any bridge argument literal.
        self.assertNotIn(
            '"pose/info"',
            source,
            "sim.py must not bridge .../pose/info to /tf.",
        )


class TestArcoScenarioObstacleTF(unittest.TestCase):
    """Verify that arco_scenario.py publishes static obstacle TF frames."""

    def _read(self):
        with open(_SCENARIO_LAUNCH, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_uses_static_transform_publisher(self):
        """arco_scenario.py must instantiate static_transform_publisher nodes."""
        source = self._read()
        # The executable string must appear as a Node argument, not only in
        # a comment or docstring.
        self.assertIn(
            '"static_transform_publisher"',
            source,
            "arco_scenario.py must publish static obstacle TF frames via "
            "a Node with executable='static_transform_publisher'.",
        )

    def test_all_obstacle_frame_ids_declared(self):
        """Each obstacle name must appear as a string literal (child-frame-id)."""
        source = self._read()
        for name in EXPECTED_OBSTACLE_NAMES:
            with self.subTest(obstacle=name):
                # Names are defined in the _OBSTACLE_FRAMES constant as string
                # literals and passed as --child-frame-id at runtime.
                self.assertIn(
                    f'"{name}"',
                    source,
                    f"arco_scenario.py must declare static TF frame '{name}' "
                    "as a string literal.",
                )

    def test_obstacle_uses_child_frame_id_argument(self):
        """The --child-frame-id flag must appear in the static_transform_publisher args."""
        source = self._read()
        self.assertIn(
            '"--child-frame-id"',
            source,
            "static_transform_publisher must use --child-frame-id argument.",
        )

    def test_obstacle_parent_frame_is_world(self):
        """Obstacle static transforms must be relative to the 'world' frame."""
        source = self._read()
        # The world frame is passed as --frame-id argument value.
        self.assertIn(
            '"--frame-id", "world"',
            source,
            "Obstacle TF parent must be set to 'world' via --frame-id.",
        )


if __name__ == "__main__":
    unittest.main()
