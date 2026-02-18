"""Robot model resolution logic for FRET launchers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import logging
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

logger = logging.get_logger("fret.model")


def resolve_robot_model(model_name: str, fret_share: str) -> dict:
    """
    Resolves robot model across fallback sources.

    Searches for robot descriptions in the following order:
    1. Local URDF file: share/fret/urdf/<model>.urdf
    2. Local XACRO file: share/fret/urdf/<model>.xacro
    3. External ROS package: <model>_description (or ur_description for UR models)

    Args:
        model_name: Robot model name (e.g., 'scara', 'ur3')
        fret_share: Path to fret package share directory

    Returns:
        Dictionary with key 'robot_description' containing the robot description.
        The value is either a string (URDF) or Command object (XACRO compilation).

    Raises:
        ValueError: If model cannot be resolved from any source
    """
    urdf_path = os.path.join(fret_share, "urdf", f"{model_name}.urdf")
    xacro_path = os.path.join(fret_share, "urdf", f"{model_name}.xacro")

    robot_description_content = None

    # Try local URDF
    if os.path.exists(urdf_path):
        logger.info(f"Found URDF: {urdf_path}")
        with open(urdf_path, "r", encoding="utf-8") as urdf_file:
            robot_description_content = urdf_file.read()

    # Try local XACRO
    elif os.path.exists(xacro_path):
        logger.info(f"Found XACRO: {xacro_path}")
        robot_description_content = Command(
            [FindExecutable(name="xacro"), " ", xacro_path]
        )

    # Try external ROS package (UR as fallback)
    elif model_name.lower().startswith("ur"):
        logger.info(
            f"Found external package: Universal Robots; Model: {model_name}"
        )
        ur_description_share = get_package_share_directory("ur_description")
        description_file = f"{ur_description_share}/urdf/ur.urdf.xacro"

        # Get UR-specific configuration from launch context
        safety_limits = LaunchConfiguration("safety_limits")
        safety_pos_margin = LaunchConfiguration("safety_pos_margin")
        safety_k_position = LaunchConfiguration("safety_k_position")
        tf_prefix = LaunchConfiguration("tf_prefix")

        robot_description_content = Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(description_file),
                " safety_limits:=",
                safety_limits,
                " safety_pos_margin:=",
                safety_pos_margin,
                " safety_k_position:=",
                safety_k_position,
                " name:=ur",
                " ur_type:=",
                model_name,
                " tf_prefix:=",
                tf_prefix,
            ]
        )

    else:
        raise ValueError(f"Unsupported model specified: {model_name}")

    if robot_description_content is None:
        raise ValueError(
            f"Failed to resolve robot description for model: {model_name}"
        )

    return {"robot_description": robot_description_content}
