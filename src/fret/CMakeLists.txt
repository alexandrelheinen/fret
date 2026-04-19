cmake_minimum_required(VERSION 3.8)
project(fret)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(GENERATED_URDF_DIR   ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GENERATED_MESHES_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_meshes)
file(MAKE_DIRECTORY ${GENERATED_URDF_DIR})

# ---------------------------------------------------------------------------
# Per-model loop — driven by urdf/*.xacro
# ---------------------------------------------------------------------------
file(GLOB XACRO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro)

set(GENERATED_URDF_FILES)
set(MESH_STAMPS)

foreach(XACRO_FILE ${XACRO_FILES})
  get_filename_component(MODEL_NAME ${XACRO_FILE} NAME_WE)
  message(STATUS "[fret] model: ${MODEL_NAME}")

  # -- URDF from XACRO -------------------------------------------------------
  set(URDF_FILE ${GENERATED_URDF_DIR}/${MODEL_NAME}.urdf)
  add_custom_command(
    OUTPUT  ${URDF_FILE}
    COMMAND ${XACRO_EXECUTABLE} ${XACRO_FILE} -o ${URDF_FILE}
    DEPENDS ${XACRO_FILE}
    COMMENT "[fret] ${MODEL_NAME}: XACRO → URDF"
    VERBATIM
  )
  list(APPEND GENERATED_URDF_FILES ${URDF_FILE})

  # -- Mesh generator (optional: mesh/<model>.py) ----------------------------
  set(MESH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/mesh/${MODEL_NAME}.py)
  if(EXISTS ${MESH_SCRIPT})
    message(STATUS "[fret]   mesh generator : mesh/${MODEL_NAME}.py")
    set(MODEL_MESH_DIR ${GENERATED_MESHES_DIR}/${MODEL_NAME})
    set(MESH_STAMP     ${GENERATED_MESHES_DIR}/${MODEL_NAME}.stamp)
    add_custom_command(
      OUTPUT  ${MESH_STAMP}
      COMMAND ${Python3_EXECUTABLE} ${MESH_SCRIPT} --output-dir ${MODEL_MESH_DIR}
      COMMAND ${CMAKE_COMMAND} -E touch ${MESH_STAMP}
      DEPENDS ${MESH_SCRIPT} ${XACRO_FILE}
      COMMENT "[fret] ${MODEL_NAME}: generating meshes"
      VERBATIM
    )
    list(APPEND MESH_STAMPS ${MESH_STAMP})
    install(
      DIRECTORY ${MODEL_MESH_DIR}/
      DESTINATION share/${PROJECT_NAME}/meshes/${MODEL_NAME}
      OPTIONAL
    )
  else()
    message(STATUS "[fret]   mesh generator : (none)")
  endif()

  # -- RViz config (optional: rviz/<model>.rviz) -----------------------------
  set(RVIZ_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rviz/${MODEL_NAME}.rviz)
  if(EXISTS ${RVIZ_FILE})
    message(STATUS "[fret]   RViz config    : rviz/${MODEL_NAME}.rviz")
  else()
    message(STATUS "[fret]   RViz config    : (none, using default.rviz)")
  endif()

endforeach()

# ---------------------------------------------------------------------------
# Top-level build targets
# ---------------------------------------------------------------------------
add_custom_target(generate_urdfs  ALL DEPENDS ${GENERATED_URDF_FILES})
add_custom_target(generate_meshes ALL DEPENDS ${MESH_STAMPS})

# ---------------------------------------------------------------------------
# Install generated URDFs and XACRO source files.
# sim.py calls `xacro <pkg_share>/urdf/<model>.xacro` at launch time, so
# the source .xacro files must be present in the install tree alongside the
# pre-generated .urdf files.
# ---------------------------------------------------------------------------
install(FILES ${GENERATED_URDF_FILES}
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro"
)

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

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(DIRECTORY worlds
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
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/arc_injector
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/straight_line_injector
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/perception_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Install validation scripts
# ---------------------------------------------------------------------------
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/validate_occupancy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
