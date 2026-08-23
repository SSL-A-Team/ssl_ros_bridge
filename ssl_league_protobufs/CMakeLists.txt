cmake_minimum_required(VERSION 3.5)
project(ssl_league_protobufs)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

# ssl-protocol-defs/ is the RoboCup-SSL/ssl-protocol-defs submodule. It ships a
# CMake module that runs protoc with a single --proto_path root, which is what
# the cross-imports in these protos require; see the comments in
# SSLProtocolDefs.cmake for why CMake's own protobuf_generate_cpp() cannot be
# used here. PROTO_ROOT and the output layout are derived automatically, and
# OUT_DIR defaults to ${CMAKE_CURRENT_BINARY_DIR}.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/ssl-protocol-defs/cmake")
include(SSLProtocolDefs)

ssl_protocol_defs_generate_cpp(
  PROTOS
    gamecontroller/ssl_gc_common.proto
    gamecontroller/ssl_gc_game_event.proto
    gamecontroller/ssl_gc_geometry.proto
    gamecontroller/ssl_gc_rcon_autoref.proto
    gamecontroller/ssl_gc_rcon_remotecontrol.proto
    gamecontroller/ssl_gc_rcon_team.proto
    gamecontroller/ssl_gc_rcon.proto
    gamecontroller/ssl_gc_referee_message.proto
    simulation/ssl_simulation_config.proto
    simulation/ssl_simulation_control.proto
    simulation/ssl_simulation_error.proto
    simulation/ssl_simulation_robot_control.proto
    simulation/ssl_simulation_robot_feedback.proto
    simulation/ssl_simulation_synchronous.proto
    vision/ssl_vision_detection_tracked.proto
    vision/ssl_vision_detection.proto
    vision/ssl_vision_geometry.proto
    vision/ssl_vision_wrapper_tracked.proto
    vision/ssl_vision_wrapper.proto
  SOURCES_VAR PROTOBUF_SRCS
  HEADERS_VAR PROTOBUF_HDRS
)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_HDRS}
  ${PROTOBUF_SRCS}
)
target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    # Generated headers live under subdirectories (gamecontroller/, vision/,
    # simulation/)
    # that mirror ssl-protocol-defs' own layout, and they include each other
    # by path from that root (e.g. "gamecontroller/ssl_gc_common.pb.h"). PUBLIC,
    # not INTERFACE, because this package's own compilation needs that root
    # too, not just external consumers, to resolve those same cross-includes.
    # Two include roots are exported: the normal <ssl_league_protobufs/...>
    # root every consumer's top-level include expects, and this package's
    # own generated-header root, needed to resolve the cross-includes at
    # both build and install time.
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Disable noisy deprecation warnings we can't control from league protobufs
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

if(BUILD_TESTING)
  # ssl-protocol-defs/ is a vendored third-party submodule. Its files are
  # not ours to lint or copyright-check. AMENT_LINT_AUTO_FILE_EXCLUDE
  # correctly excludes it for flake8, cpplint, cppcheck, and uncrustify.
  # ament_cmake_pep257's hook has no exclude support at all; it always scans
  # every *.py file recursively. ament_copyright's exclude only matches
  # exact file paths, not a directory prefix. This package has no Python or
  # copyright-checked source of its own, so both linters are skipped
  # entirely rather than left to flag the submodule's files.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/ssl-protocol-defs")
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257 ament_cmake_copyright)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
