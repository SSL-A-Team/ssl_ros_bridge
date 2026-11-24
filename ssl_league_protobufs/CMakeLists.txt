cmake_minimum_required(VERSION 3.5)
project(ssl_league_protobufs)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ssl-protocol-defs/cmake")
include(SSLProtocolDefs)

set(GENERATED_SRCS_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/sslproto")

set(PROTOS
  gamecontroller/ssl_gc_common.proto
  gamecontroller/ssl_gc_game_event.proto
  gamecontroller/ssl_gc_geometry.proto
  gamecontroller/ssl_gc_rcon_autoref.proto
  gamecontroller/ssl_gc_rcon_remotecontrol.proto
  gamecontroller/ssl_gc_rcon_team.proto
  gamecontroller/ssl_gc_rcon.proto
  gamecontroller/ssl_gc_referee_message.proto

  simulation/erforce/ssl_simulation_custom_erforce_realism.proto
  simulation/erforce/ssl_simulation_custom_erforce_robot_spec.proto
  simulation/ssl_simulation_config.proto
  simulation/ssl_simulation_control.proto
  simulation/ssl_simulation_error.proto
  simulation/ssl_simulation_robot_control.proto
  simulation/ssl_simulation_robot_feedback.proto
  simulation/ssl_simulation_synchronous.proto

  vision/legacy/messages_robocup_ssl_geometry_legacy.proto
  vision/legacy/messages_robocup_ssl_wrapper_legacy.proto
  vision/ssl_vision_detection_tracked.proto
  vision/ssl_vision_detection.proto
  vision/ssl_vision_geometry.proto
  vision/ssl_vision_wrapper_tracked.proto
  vision/ssl_vision_wrapper.proto
)

ssl_protocol_defs_generate_cpp(
        OUT_DIR ${GENERATED_SRCS_DIR}
        PROTOS ${PROTOS}
        SOURCES_VAR PROTO_SRCS
        HEADERS_VAR PROTO_HDRS)

add_custom_target(GENERATE_PROTOS DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})


add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE ${PROTO_SRCS})
add_dependencies(${PROJECT_NAME} GENERATE_PROTOS)
target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${GENERATED_SRCS_DIR}>
    $<INSTALL_INTERFACE:include>
)
# Disable noisy deprecation warnings we can't control from league protobufs
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)


ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)

install(
  DIRECTORY ${GENERATED_SRCS_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
