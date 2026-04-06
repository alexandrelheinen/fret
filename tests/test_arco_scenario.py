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
    "obstacle_pillar_left",
    "obstacle_pillar_right",
    "obstacle_crossbar",
    "obstacle_block_a",
    "obstacle_block_b",
}

EXPECTED_REGION_NAMES = {
    "region_start",
    "region_target",
}

MIN_OBSTACLE_COUNT = 5


# ---------------------------------------------------------------------------
# World SDF tests
# ---------------------------------------------------------------------------


class TestArcoScenarioWorldFile(unittest.TestCase):
    """Structural validation of the ARCO scenario SDF world file."""

    def _parse(self):
        """Return (root, world) element pair; fail fast if file is missing."""
        self.assertTrue(
            os.path.isfile(_WORLD_FILE),
            f"World file not found: {_WORLD_FILE}",
        )
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

    def test_obstacle_count_at_least_five(self):
        """FR-02 requires at least five obstacle primitives."""
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

    def test_launch_file_uses_ur3_model(self):
        """arco_scenario.py must default to the ur3 robot model."""
        source = self._read(_SCENARIO_LAUNCH)
        self.assertIn(
            "ur3",
            source,
            "arco_scenario.py must reference the ur3 robot model.",
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

    def test_world_default_is_empty_sdf(self):
        """'world' argument default must be 'empty.sdf' for backward compat."""
        source = self._read()
        self.assertIn(
            "empty.sdf",
            source,
            "Default world must remain 'empty.sdf' for backward compatibility.",
        )


if __name__ == "__main__":
    unittest.main()
