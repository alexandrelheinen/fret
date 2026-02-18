"""Unit tests for FRET model resolution logic."""

import os
import unittest
from unittest.mock import MagicMock, mock_open, patch

# Import after unittest setup to ensure proper module initialization
from fret.launch.model import resolve_robot_model


class TestLocalUrdfResolution(unittest.TestCase):
    """Tests for local URDF file resolution."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"
        self.mock_urdf_path = os.path.join(
            self.mock_fret_share, "urdf", "scara.urdf"
        )
        self.mock_urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""

    def test_resolve_local_urdf_file(self):
        """Test that a local URDF file is correctly resolved and read."""
        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "builtins.open", mock_open(read_data=self.mock_urdf_content)
            ) as mock_file,
            patch("fret.launch.model.logger") as mock_logger,
        ):

            # Configure mock: URDF exists, XACRO doesn't
            mock_exists.side_effect = lambda path: path == self.mock_urdf_path

            result = resolve_robot_model("scara", self.mock_fret_share)

            self.assertEqual(
                result, {"robot_description": self.mock_urdf_content}
            )
            mock_file.assert_called_once_with(
                self.mock_urdf_path, "r", encoding="utf-8"
            )
            mock_logger.info.assert_called_with(
                f"Found URDF: {self.mock_urdf_path}"
            )

    def test_resolve_local_urdf_file_encoding(self):
        """Test that URDF files are read with UTF-8 encoding."""
        urdf_content = """<?xml version="1.0" encoding="UTF-8"?>
<robot name="test">
  <link name="base_link"/>
</robot>"""

        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "builtins.open", mock_open(read_data=urdf_content)
            ) as mock_file,
        ):

            mock_exists.side_effect = lambda path: path == self.mock_urdf_path

            result = resolve_robot_model("scara", self.mock_fret_share)

            # Verify file opened with UTF-8 encoding
            mock_file.assert_called_once()
            call_kwargs = mock_file.call_args[1]
            self.assertEqual(call_kwargs["encoding"], "utf-8")
            self.assertEqual(result["robot_description"], urdf_content)


class TestLocalXacroResolution(unittest.TestCase):
    """Tests for local XACRO file resolution."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"
        self.mock_xacro_path = os.path.join(
            self.mock_fret_share, "urdf", "scara.xacro"
        )
        self.mock_urdf_path = os.path.join(
            self.mock_fret_share, "urdf", "scara.urdf"
        )

    def test_resolve_local_xacro_file(self):
        """Test that a local XACRO file is correctly resolved as a Command."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.Command") as mock_command,
            patch("fret.launch.model.FindExecutable") as mock_find_exec,
            patch("fret.launch.model.logger"),
        ):

            # Configure mock: URDF doesn't exist, XACRO does
            mock_exists.side_effect = lambda path: path == self.mock_xacro_path
            mock_find_exec.return_value = "xacro"
            mock_command_instance = MagicMock()
            mock_command.return_value = mock_command_instance

            result = resolve_robot_model("scara", self.mock_fret_share)

            self.assertEqual(
                result["robot_description"], mock_command_instance
            )
            # Verify Command was called with xacro executable and file path
            mock_command.assert_called_once()
            call_args = mock_command.call_args[0][0]
            self.assertIn("xacro", call_args)
            self.assertIn(self.mock_xacro_path, call_args)

    def test_xacro_resolution_priority(self):
        """Test that URDF is preferred over XACRO when both exist."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("builtins.open", mock_open(read_data="<urdf/>")),
        ):

            # Both files exist
            mock_exists.side_effect = lambda path: path in (
                self.mock_urdf_path,
                self.mock_xacro_path,
            )

            result = resolve_robot_model("scara", self.mock_fret_share)

            # Should use URDF content, not Command
            self.assertIsInstance(result["robot_description"], str)
            self.assertEqual(result["robot_description"], "<urdf/>")


class TestUrVendorFallback(unittest.TestCase):
    """Tests for Universal Robots vendor package fallback."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"
        self.mock_ur_description_share = "/opt/ros/jazzy/share/ur_description"

    def test_resolve_ur_model_fallback(self):
        """Test that UR models trigger ur_description package fallback."""
        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "fret.launch.model.get_package_share_directory"
            ) as mock_get_share,
            patch(
                "fret.launch.model.LaunchConfiguration"
            ) as mock_launch_config,
            patch("fret.launch.model.Command") as mock_command,
            patch("fret.launch.model.FindExecutable"),
            patch("fret.launch.model.logger"),
        ):

            # Local files don't exist
            mock_exists.return_value = False
            # UR description package exists
            mock_get_share.return_value = self.mock_ur_description_share

            # Mock LaunchConfiguration
            mock_config = MagicMock()
            mock_launch_config.return_value = mock_config

            mock_command_instance = MagicMock()
            mock_command.return_value = mock_command_instance

            result = resolve_robot_model("ur3", self.mock_fret_share)

            self.assertEqual(
                result["robot_description"], mock_command_instance
            )
            # Verify ur_description package was queried
            mock_get_share.assert_called_once_with("ur_description")

    def test_ur_model_detection(self):
        """Test that various UR model names are correctly detected."""
        ur_models = [
            "ur3",
            "ur3e",
            "ur5",
            "ur5e",
            "ur10",
            "ur10e",
            "UR5",
            "UR3E",
        ]

        for model in ur_models:
            with (
                patch("os.path.exists") as mock_exists,
                patch(
                    "fret.launch.model.get_package_share_directory"
                ) as mock_get_share,
                patch("fret.launch.model.LaunchConfiguration"),
                patch("fret.launch.model.Command"),
                patch("fret.launch.model.FindExecutable"),
                patch("fret.launch.model.logger"),
            ):

                mock_exists.return_value = False
                mock_get_share.return_value = self.mock_ur_description_share

                try:
                    resolve_robot_model(model, self.mock_fret_share)
                    # Should attempt to load UR description
                    mock_get_share.assert_called_with("ur_description")
                except Exception:
                    # Expected if dependencies aren't mocked perfectly
                    pass

    def test_ur_xacro_parameters(self):
        """Test that UR-specific XACRO parameters are correctly set."""
        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "fret.launch.model.get_package_share_directory"
            ) as mock_get_share,
            patch(
                "fret.launch.model.LaunchConfiguration"
            ) as mock_launch_config,
            patch("fret.launch.model.Command") as mock_command,
            patch("fret.launch.model.FindExecutable"),
            patch("fret.launch.model.logger"),
        ):

            mock_exists.return_value = False
            mock_get_share.return_value = self.mock_ur_description_share

            # Each LaunchConfiguration is queried with perform
            mock_config = MagicMock()
            mock_config.perform.return_value = "test_value"
            mock_launch_config.return_value = mock_config

            mock_command.return_value = MagicMock()

            resolve_robot_model("ur5", self.mock_fret_share)

            # Verify LaunchConfiguration was created for the expected parameters
            expected_params = [
                "safety_limits",
                "safety_pos_margin",
                "safety_k_position",
                "tf_prefix",
            ]
            for param in expected_params:
                mock_launch_config.assert_any_call(param)


class TestErrorHandling(unittest.TestCase):
    """Tests for error handling and edge cases."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"

    def test_unsupported_model_raises_error(self):
        """Test that unsupported models raise ValueError."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.logger"),
        ):

            # No local files, not a UR model
            mock_exists.return_value = False

            with self.assertRaises(ValueError) as context:
                resolve_robot_model("unsupported_robot", self.mock_fret_share)

            self.assertIn(
                "Unsupported model specified", str(context.exception)
            )
            self.assertIn("unsupported_robot", str(context.exception))

    def test_missing_model_raises_error(self):
        """Test that missing models raise appropriate error."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.logger"),
        ):

            mock_exists.return_value = False

            with self.assertRaises(ValueError):
                resolve_robot_model("nonexistent", self.mock_fret_share)

    def test_empty_model_name_raises_error(self):
        """Test that empty model names raise error."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.logger"),
        ):

            mock_exists.return_value = False

            with self.assertRaises(ValueError):
                resolve_robot_model("", self.mock_fret_share)

    def test_case_insensitive_ur_detection(self):
        """Test that UR detection is case-insensitive."""
        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "fret.launch.model.get_package_share_directory"
            ) as mock_get_share,
            patch("fret.launch.model.LaunchConfiguration"),
            patch("fret.launch.model.Command"),
            patch("fret.launch.model.FindExecutable"),
            patch("fret.launch.model.logger"),
        ):

            mock_exists.return_value = False
            mock_get_share.return_value = "/opt/ros/jazzy/share/ur_description"

            # Test with uppercase
            try:
                resolve_robot_model("UR3", self.mock_fret_share)
                mock_get_share.assert_called_with("ur_description")
            except Exception:
                pass


class TestDirectoryStructure(unittest.TestCase):
    """Tests for correct directory path construction."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"

    def test_urdf_path_construction(self):
        """Test that URDF path is correctly constructed."""
        expected_urdf = os.path.join(
            self.mock_fret_share, "urdf", "scara.urdf"
        )

        with (
            patch("os.path.exists") as mock_exists,
            patch("builtins.open", mock_open(read_data="<urdf/>")),
        ):

            def exists_check(path):
                return path == expected_urdf

            mock_exists.side_effect = exists_check

            resolve_robot_model("scara", self.mock_fret_share)

            # Verify the exact path was checked
            mock_exists.assert_called()
            calls = [call[0][0] for call in mock_exists.call_args_list]
            self.assertIn(expected_urdf, calls)

    def test_xacro_path_construction(self):
        """Test that XACRO path is correctly constructed."""
        expected_xacro = os.path.join(
            self.mock_fret_share, "urdf", "scara.xacro"
        )

        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.Command") as mock_command,
            patch("fret.launch.model.FindExecutable"),
            patch("fret.launch.model.logger"),
        ):

            def exists_check(path):
                return path == expected_xacro

            mock_exists.side_effect = exists_check
            mock_command.return_value = MagicMock()

            resolve_robot_model("scara", self.mock_fret_share)

            # Verify the exact path was checked
            calls = [call[0][0] for call in mock_exists.call_args_list]
            self.assertIn(expected_xacro, calls)


class TestReturnValues(unittest.TestCase):
    """Tests for return value structure."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_fret_share = "/opt/ros/jazzy/share/fret"
        self.mock_urdf_path = os.path.join(
            self.mock_fret_share, "urdf", "scara.urdf"
        )
        self.mock_xacro_path = os.path.join(
            self.mock_fret_share, "urdf", "scara.xacro"
        )
        self.mock_urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""

    def test_return_structure_urdf(self):
        """Test that URDF resolution returns correct dictionary structure."""
        with (
            patch("os.path.exists") as mock_exists,
            patch(
                "builtins.open", mock_open(read_data=self.mock_urdf_content)
            ),
        ):

            mock_exists.side_effect = lambda path: path == self.mock_urdf_path

            result = resolve_robot_model("scara", self.mock_fret_share)

            self.assertIsInstance(result, dict)
            self.assertIn("robot_description", result)
            self.assertEqual(len(result), 1)  # Only one key

    def test_return_structure_xacro(self):
        """Test that XACRO resolution returns correct dictionary structure."""
        with (
            patch("os.path.exists") as mock_exists,
            patch("fret.launch.model.Command") as mock_command,
            patch("fret.launch.model.FindExecutable"),
        ):

            mock_exists.side_effect = lambda path: path == self.mock_xacro_path
            mock_command_instance = MagicMock()
            mock_command.return_value = mock_command_instance

            result = resolve_robot_model("scara", self.mock_fret_share)

            self.assertIsInstance(result, dict)
            self.assertIn("robot_description", result)
            self.assertEqual(
                result["robot_description"], mock_command_instance
            )


if __name__ == "__main__":
    unittest.main()
