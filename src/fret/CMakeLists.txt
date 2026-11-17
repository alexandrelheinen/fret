
cmake_minimum_required(VERSION 3.8)
project(fret)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# ---------------------------------------------------------------------------
# Install Python package (fret) into ament index
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
)

# ament_python_install_package installs to site-packages only, but stale builds
# may leave a __init__.py in share/${PROJECT_NAME}/ which causes Python to find
# an incomplete "fret" package there instead of from site-packages.  Remove it.
install(CODE
  "file(REMOVE \"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/__init__.py\")"
)

# ---------------------------------------------------------------------------
# Install ROS 2 resource directories
# ---------------------------------------------------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mjcf)
  install(DIRECTORY mjcf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ---------------------------------------------------------------------------
# Environment hook (adds src/ to PYTHONPATH)
# ---------------------------------------------------------------------------
ament_environment_hooks(hooks/fret.sh)

# ---------------------------------------------------------------------------
# Install Python node entry-point scripts
# ---------------------------------------------------------------------------
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/controller_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/scene_acquisition_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/planner_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/perception_bridge
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_bridge
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/dubins_race_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
