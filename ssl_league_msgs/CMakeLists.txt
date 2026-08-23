cmake_minimum_required(VERSION 3.18)
project(ssl_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include(cmake/Ros2MsgGen.cmake)

set(_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ssl_league_protobufs/ssl-protocol-defs/proto")

generate_ros2_msgs(
  PROTO_FILES
    ${_PROTO_DIR}/gamecontroller/ssl_gc_common.proto
    ${_PROTO_DIR}/gamecontroller/ssl_gc_geometry.proto
    ${_PROTO_DIR}/gamecontroller/ssl_gc_game_event.proto
    ${_PROTO_DIR}/gamecontroller/ssl_gc_referee_message.proto
    ${_PROTO_DIR}/gamecontroller/ssl_gc_rcon.proto
    ${_PROTO_DIR}/vision/ssl_vision_detection.proto
    ${_PROTO_DIR}/vision/ssl_vision_geometry.proto
    ${_PROTO_DIR}/vision/ssl_vision_wrapper.proto
    ${_PROTO_DIR}/simulation/ssl_simulation_config.proto
    ${_PROTO_DIR}/simulation/ssl_simulation_control.proto
    ${_PROTO_DIR}/simulation/ssl_simulation_error.proto
  PROTO_PATHS
    ${_PROTO_DIR}
  SIDECAR
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ssl_ros_annotations.json
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${GENERATED_ROS2_MSGS}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_proto_shared test/test_proto_shared.py)
  ament_add_pytest_test(test_protoc_gen_ros2msg test/test_protoc_gen_ros2msg.py)
endif()

ament_package()
