"""Robot model resolution logic for FRET launchers."""

import os

from launch import logging
from launch.substitutions import Command, FindExecutable

logger = logging.get_logger("fret.model")


def resolve_robot_model(model_name: str, fret_share: str) -> dict:
    """
    Resolves robot model across fallback sources.

    Searches for robot descriptions in the following order:
    1. Pre-built URDF file: share/fret/urdf/<model>.urdf
    2. Local XACRO file: share/fret/urdf/<model>.xacro

    Args:
        model_name: Robot model name (e.g., 'scara')
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

    # Try pre-built URDF (generated at build time from the XACRO)
    if os.path.exists(urdf_path):
        logger.info(f"Found URDF: {urdf_path}")
        with open(urdf_path, "r", encoding="utf-8") as urdf_file:
            robot_description_content = urdf_file.read()

    # Try local XACRO (compiled at launch time)
    elif os.path.exists(xacro_path):
        logger.info(f"Found XACRO: {xacro_path}")
        robot_description_content = Command(
            [FindExecutable(name="xacro"), " ", xacro_path]
        )

    else:
        raise ValueError(f"Unsupported model specified: {model_name}")

    if robot_description_content is None:
        raise ValueError(
            f"Failed to resolve robot description for model: {model_name}"
        )

    return {"robot_description": robot_description_content}
