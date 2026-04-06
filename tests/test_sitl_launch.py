"""Tests for the SITL end-to-end pipeline launcher (Issue 08).

Validates:
- sitl.py launch file existence and Python validity.
- Presence of required launch arguments (scenario, record_bag, bag_dir).
- Default scenario argument matches arco_scenario.
- Default record_bag is false (bag recording is opt-in).
- sitl.py references the correct sub-launcher pattern.
- sitl.yaml configuration file existence and YAML validity.
- sitl.yaml contains the expected top-level keys.
- sitl.yaml planner and replanning profile keys are present.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import ast
import os
import sys
import unittest

try:
    import yaml

    _YAML_AVAILABLE = True
except ImportError:  # pragma: no cover – yaml always available in test env
    _YAML_AVAILABLE = False

# ---------------------------------------------------------------------------
# Paths resolved relative to the repository root
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
_SRC_FRET = os.path.join(_SRC_DIR, "fret")

# Make the fret package importable without a full ROS 2 build
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
_SITL_LAUNCH = os.path.join(_SRC_FRET, "launch", "sitl.py")
_SITL_CONFIG = os.path.join(_SRC_FRET, "config", "sitl.yaml")

# ---------------------------------------------------------------------------
# Expected constants
# ---------------------------------------------------------------------------
EXPECTED_LAUNCH_ARGS = {"scenario", "record_bag", "bag_dir"}
DEFAULT_SCENARIO = "arco_scenario"
DEFAULT_RECORD_BAG = "false"
DEFAULT_BAG_DIR = "log/bags"

EXPECTED_SITL_YAML_KEYS = {"sitl"}
EXPECTED_PLANNER_KEYS = {"algorithm", "timeout", "rng_seed", "rrt_connect"}
EXPECTED_REPLANNING_KEYS = {
    "min_replan_interval",
    "path_invalidation_ratio",
    "tracking_error_threshold",
}


# ---------------------------------------------------------------------------
# Launch file tests
# ---------------------------------------------------------------------------


class TestSitlLaunchFileExists(unittest.TestCase):
    """File-level checks for sitl.py."""

    def test_launch_file_exists(self):
        """sitl.py must exist in the launch directory."""
        self.assertTrue(
            os.path.isfile(_SITL_LAUNCH),
            f"sitl.py not found: {_SITL_LAUNCH}",
        )

    def test_launch_file_is_valid_python(self):
        """sitl.py must be syntactically valid Python."""
        with open(_SITL_LAUNCH, "r", encoding="utf-8") as fh:
            source = fh.read()
        try:
            ast.parse(source)
        except SyntaxError as exc:
            self.fail(f"sitl.py has a syntax error: {exc}")

    def test_launch_file_defines_generate_launch_description(self):
        """sitl.py must define the generate_launch_description entry point."""
        with open(_SITL_LAUNCH, "r", encoding="utf-8") as fh:
            source = fh.read()
        self.assertIn(
            "generate_launch_description",
            source,
            "sitl.py must define generate_launch_description().",
        )


class TestSitlLaunchArguments(unittest.TestCase):
    """Launch argument checks for sitl.py."""

    def _source(self):
        with open(_SITL_LAUNCH, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_scenario_argument_declared(self):
        """sitl.py must declare the 'scenario' launch argument."""
        source = self._source()
        self.assertIn(
            '"scenario"',
            source,
            "sitl.py must declare a 'scenario' launch argument.",
        )

    def test_record_bag_argument_declared(self):
        """sitl.py must declare the 'record_bag' launch argument."""
        source = self._source()
        self.assertIn(
            '"record_bag"',
            source,
            "sitl.py must declare a 'record_bag' launch argument.",
        )

    def test_bag_dir_argument_declared(self):
        """sitl.py must declare the 'bag_dir' launch argument."""
        source = self._source()
        self.assertIn(
            '"bag_dir"',
            source,
            "sitl.py must declare a 'bag_dir' launch argument.",
        )

    def test_default_scenario_is_arco_scenario(self):
        """Default scenario must be 'arco_scenario'."""
        source = self._source()
        self.assertIn(
            DEFAULT_SCENARIO,
            source,
            f"sitl.py default scenario must be '{DEFAULT_SCENARIO}'.",
        )

    def test_default_record_bag_is_false(self):
        """Bag recording must default to false (opt-in only)."""
        source = self._source()
        self.assertIn(
            '"false"',
            source,
            "sitl.py must default record_bag to 'false'.",
        )

    def test_default_bag_dir_value(self):
        """Default bag directory must be 'log/bags'."""
        source = self._source()
        self.assertIn(
            DEFAULT_BAG_DIR,
            source,
            f"sitl.py must set default bag_dir to '{DEFAULT_BAG_DIR}'.",
        )


class TestSitlLaunchContent(unittest.TestCase):
    """Content checks for sitl.py launch composition."""

    def _source(self):
        with open(_SITL_LAUNCH, "r", encoding="utf-8") as fh:
            return fh.read()

    def test_includes_scenario_sublauncher(self):
        """sitl.py must use IncludeLaunchDescription to compose a sub-launcher."""
        source = self._source()
        self.assertIn(
            "IncludeLaunchDescription",
            source,
            "sitl.py must use IncludeLaunchDescription for composition.",
        )

    def test_bag_recording_uses_ros2_bag(self):
        """sitl.py must reference 'ros2 bag record' for artifact capture."""
        source = self._source()
        self.assertIn(
            "ros2",
            source,
            "sitl.py must reference ros2 bag record for artifact capture.",
        )
        self.assertIn(
            "bag",
            source,
            "sitl.py must reference ros2 bag record for artifact capture.",
        )

    def test_opaque_function_used(self):
        """sitl.py must use OpaqueFunction for runtime argument evaluation."""
        source = self._source()
        self.assertIn(
            "OpaqueFunction",
            source,
            "sitl.py must use OpaqueFunction for runtime evaluation.",
        )

    def test_error_raised_for_missing_scenario(self):
        """sitl.py must raise an error when the scenario file is not found."""
        source = self._source()
        self.assertIn(
            "FileNotFoundError",
            source,
            "sitl.py must raise FileNotFoundError for missing scenarios.",
        )


# ---------------------------------------------------------------------------
# Config file tests
# ---------------------------------------------------------------------------


class TestSitlConfigFile(unittest.TestCase):
    """Validation of the sitl.yaml demo profile configuration."""

    def _load(self):
        """Load and return parsed YAML content."""
        with open(_SITL_CONFIG, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_config_file_exists(self):
        """sitl.yaml must exist in the config directory."""
        self.assertTrue(
            os.path.isfile(_SITL_CONFIG),
            f"sitl.yaml not found: {_SITL_CONFIG}",
        )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_config_is_valid_yaml(self):
        """sitl.yaml must be parseable YAML."""
        cfg = self._load()
        self.assertIsInstance(cfg, dict, "sitl.yaml must parse to a dict.")

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_config_has_sitl_top_level_key(self):
        """sitl.yaml must have a 'sitl' top-level mapping."""
        cfg = self._load()
        self.assertIn(
            "sitl",
            cfg,
            "sitl.yaml must contain a 'sitl' top-level key.",
        )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_sitl_has_scenario_key(self):
        """sitl.yaml must declare the default scenario name."""
        cfg = self._load()
        sitl = cfg["sitl"]
        self.assertIn(
            "scenario",
            sitl,
            "sitl.yaml['sitl'] must contain a 'scenario' key.",
        )
        self.assertEqual(
            sitl["scenario"],
            DEFAULT_SCENARIO,
            f"Default scenario must be '{DEFAULT_SCENARIO}'.",
        )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_sitl_has_planner_profile(self):
        """sitl.yaml must declare a planner profile with required keys."""
        cfg = self._load()
        sitl = cfg["sitl"]
        self.assertIn(
            "planner", sitl, "sitl.yaml must contain a 'planner' key."
        )
        planner = sitl["planner"]
        for key in EXPECTED_PLANNER_KEYS:
            with self.subTest(key=key):
                self.assertIn(
                    key,
                    planner,
                    f"sitl.yaml planner profile must contain key '{key}'.",
                )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_sitl_has_replanning_profile(self):
        """sitl.yaml must declare a replanning profile with required keys."""
        cfg = self._load()
        sitl = cfg["sitl"]
        self.assertIn(
            "replanning",
            sitl,
            "sitl.yaml must contain a 'replanning' key.",
        )
        replanning = sitl["replanning"]
        for key in EXPECTED_REPLANNING_KEYS:
            with self.subTest(key=key):
                self.assertIn(
                    key,
                    replanning,
                    f"sitl.yaml replanning profile must contain key '{key}'.",
                )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_planner_algorithm_is_supported(self):
        """sitl.yaml planner algorithm must be a supported algorithm name."""
        from fret.planning.planner_adapter import SUPPORTED_ALGORITHMS

        cfg = self._load()
        algorithm = cfg["sitl"]["planner"]["algorithm"]
        self.assertIn(
            algorithm,
            SUPPORTED_ALGORITHMS,
            f"Algorithm '{algorithm}' is not in SUPPORTED_ALGORITHMS.",
        )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_planner_rng_seed_is_fixed_integer(self):
        """sitl.yaml planner rng_seed must be a fixed integer for reproducibility."""
        cfg = self._load()
        rng_seed = cfg["sitl"]["planner"]["rng_seed"]
        self.assertIsInstance(
            rng_seed,
            int,
            "sitl.yaml planner rng_seed must be an integer for reproducibility.",
        )

    @unittest.skipUnless(_YAML_AVAILABLE, "pyyaml not installed")
    def test_bag_topics_list_is_non_empty(self):
        """sitl.yaml bag.topics must list at least one topic."""
        cfg = self._load()
        topics = cfg["sitl"]["bag"]["topics"]
        self.assertIsInstance(topics, list)
        self.assertGreater(
            len(topics),
            0,
            "sitl.yaml bag.topics must contain at least one topic.",
        )
        for topic in topics:
            with self.subTest(topic=topic):
                self.assertTrue(
                    topic.startswith("/"),
                    f"Topic '{topic}' must start with '/'.",
                )


if __name__ == "__main__":
    unittest.main()
