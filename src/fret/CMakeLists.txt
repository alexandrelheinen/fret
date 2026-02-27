
cmake_minimum_required(VERSION 3.8)
project(fret)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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
# Nodes
# ---------------------------------------------------------------------------
add_executable(controller
  src/controller_node.cpp
  src/control/controller_base.cpp
  src/control/controller_registry.cpp
  src/control/scara_controller.cpp
)
ament_target_dependencies(controller
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(FILES ${GENERATED_URDF_FILES}
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ---------------------------------------------------------------------------
# Environment hook
# ---------------------------------------------------------------------------
ament_environment_hooks(hooks/fret.sh)

ament_package()
